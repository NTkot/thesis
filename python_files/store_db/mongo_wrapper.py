import os
import sys
import math
import tqdm
import numpy as np
import folium
import tempfile
from typing import Callable
from pymongo import MongoClient
from PyQt5.QtWidgets import QApplication
from sklearn.cluster import AgglomerativeClustering

from data_process.segment import Segment
from .doc_builder import DocumentBuilder
from .doc_aggregator import DocumentAggregator
from .html_viewer import HTMLViewer


class MongoWrapper():
    def __init__(self,
                 doc_builder: DocumentBuilder,
                 doc_aggregator: DocumentAggregator,
                 server_options: dict = {},
                 clustering_options: dict = {'distance_threshold': 2.0},
                 store_options: dict = {'db': 'theses',
                                        'raw_collection': 'unclustered',
                                        'main_collection': 'clustered'}):
        self.mongoc = MongoClient(**server_options)
        
        if (not 'lat' in doc_builder.doc_format.keys()) or (not 'lon' in doc_builder.doc_format.keys()):
            raise ValueError("DocumentBuilder 'doc_format' dictionary needs to contain 'lat' and 'lon' keys")
        
        if (not 'severity' in doc_builder.doc_format.keys()):
            raise ValueError("DocumentBuilder 'doc_format' dictionary needs to contain 'severity'")
        
        if doc_builder.doc_format.keys() != doc_aggregator.doc_format.keys():
            raise ValueError("DocumentBuilder and DocumentAggregator do not have same keys")
        
        self.doc_builder = doc_builder
        self.doc_aggregator = doc_aggregator
        
        self.clustering_options = clustering_options
        self.store_options = store_options

    def add(self, data_segments: list[Segment]) -> bool:
        docs = []
        with tqdm.tqdm(total=len(data_segments)) as pbar:
            for data_seg in data_segments:
                docs.append(self.doc_builder.transform(data_seg))
                pbar.update(1)
        docs_with_id = self._append_id_to_docs(docs)
        self.mongoc[self.store_options["db"]][self.store_options["raw_collection"]].insert_many(docs_with_id)
        self._cluster_raw_collection()

    def add_slices(self, data: Segment,
                         slices_range: list[tuple[int, int]],
                         preprocess_data: list[tuple[list[list[str]], Callable]] = []) -> bool:
        docs = self.doc_builder.transform_slices(data, slices_range, preprocess_data)
        docs_with_id = self._append_id_to_docs(docs)
        self.mongoc[self.store_options["db"]][self.store_options["raw_collection"]].insert_many(docs_with_id)
        self._cluster_raw_collection()

    def _append_id_to_docs(self, docs: list[dict]) -> list[dict]:
        doc_count = self.mongoc[self.store_options["db"]][self.store_options["raw_collection"]].count_documents({})
        out_docs = []
        for doc in docs:
            doc["id"] = doc_count
            doc_count += 1
            out_docs.append(doc)
        return out_docs
    
    def _get_plot_markers(self, 
                          raw_db: bool, 
                          severity_based_icon: bool,
                          custom_marker=None) -> tuple[list, dict]:
        # Load docs
        if raw_db:
            docs = list(self.mongoc[self.store_options["db"]][self.store_options["raw_collection"]].find({}))
        else:
            docs = list(self.mongoc[self.store_options["db"]][self.store_options["main_collection"]].find({}))
        if not docs:
            raise RuntimeError("Database is empty, nothing to plot")
        
        # Colors
        if severity_based_icon:
            low_severity_color  = np.array([230, 230, 17])
            high_severity_color = np.array([227, 19,  11])
        else:
            icon_color = "lightred"
        
        # Find max/min severity
        min_severity = min([doc["severity"] for doc in docs])
        max_severity = max([doc["severity"] for doc in docs])

        # Create all markers by iterating each entry in docs and 
        # correctly formatting it.
        # We will iterate through all coordinates, so we can find 
        # min/max/mean during the iteration
        marker_stats = {
            'min_lat': docs[0]["lat"],
            'max_lat': docs[0]["lat"],
            'min_lon': docs[0]["lon"],
            'max_lon': docs[0]["lon"],
            'mean_lat': 0.0,
            'mean_lon': 0.0
        }
        markers = []
        for doc in docs:
            severity_relative_ratio = (doc["severity"] - min_severity) / (max_severity - min_severity)
            if severity_based_icon:
                icon_color = (severity_relative_ratio * (high_severity_color - low_severity_color) + low_severity_color).astype(int)
                icon_color = f'#{icon_color[0]:02x}{icon_color[1]:02x}{icon_color[2]:02x}'

            def to_str_with_precision(input, decimals: int):
                if hasattr(input, '__len__'):
                    if hasattr(input[0], '__len__'):
                        # 2D
                        out  = '['
                        for subarray in input:
                            out += '[' + (' '.join([f"{i:.{decimals}f}" for i in subarray]) if isinstance(subarray[0], float) else \
                                          ' '.join([f"{i:d}"   for i in subarray])) + ']'
                        out += ']'
                        return out
                    else:
                        # 1D
                        out = '[' + (' '.join([f"{i:.{decimals}f}" for i in input]) if isinstance(input[0], float) else \
                                     ' '.join([f"{i:d}"   for i in input])) + ']'
                        return out
                else:
                    # Scalar
                    return f'{input:.{decimals}f}' if isinstance(input, float) else f'{input}'

            text = f'<center> <h5>' + \
                   f'Severity: {doc["severity"]:.5f}' + \
                   f'<br>Rel. severity: {severity_relative_ratio:.3f}'
            for key in doc.keys():
                if key == 'severity' or key == '_id':
                    continue
                elif key == 'lat' or key == 'lon':
                    text += f'<br>{key.capitalize()}: {to_str_with_precision(doc[key], 10)}°'
                elif 'time' in key:
                    text += f'<br>{key.capitalize()}: {to_str_with_precision(doc[key], 2)}'
                elif key == 'length':
                    text += f'<br>{key.capitalize()}: {to_str_with_precision(doc[key], 2)}'
                else:
                    text += f'<br>{key.capitalize()}: {to_str_with_precision(doc[key], 1)}'
            text += '</h5> </center>'

            if custom_marker is not None:
                markers.append(
                    custom_marker(
                        {
                            'location': [doc["lat"], doc["lon"]], 
                            'tooltip': text,
                            'popup': text,
                        }
                    )
                )
            else:
                markers.append(folium.Marker(
                    location=[doc["lat"], doc["lon"]],
                    tooltip=text,
                    popup=text,
                    icon=folium.Icon(icon='glyphicon-exclamation-sign', 
                                     icon_color=icon_color),
                ))
            
            if doc["lat"] > marker_stats['max_lat']:
                marker_stats['max_lat'] = doc["lat"]
            elif doc["lat"] < marker_stats['min_lat']:
                marker_stats['min_lat'] = doc["lat"]
            if doc["lon"] > marker_stats['max_lon']:
                marker_stats['max_lon'] = doc["lon"]
            elif doc["lon"] < marker_stats['min_lon']:
                marker_stats['min_lon'] = doc["lon"]
            
            marker_stats['mean_lat'] += doc["lat"]
            marker_stats['mean_lon'] += doc["lon"]

        marker_stats['mean_lat'] /= len(docs)
        marker_stats['mean_lon'] /= len(docs)

        return markers, marker_stats

    def plot(self, 
             raw_db: bool, 
             severity_based_icon: bool=True,
             custom_marker=None,
             external_markers: list[folium.Marker] = None) -> None:
        # Get plot markers
        markers, marker_stats = self._get_plot_markers(raw_db=raw_db,
                                                       severity_based_icon=severity_based_icon,
                                                       custom_marker=custom_marker)

        # Initialize map and add markers
        m = folium.Map(location=(marker_stats['mean_lat'],
                                 marker_stats['mean_lon']))
        g1 = folium.FeatureGroup("Main")
        [marker.add_to(g1) for marker in markers]
        g1.add_to(m)

        # Add external markers if defined
        if external_markers is not None:
            g2 = folium.FeatureGroup("External")
            [marker.add_to(g2) for marker in external_markers]
            g2.add_to(m)

        #
        folium.LayerControl().add_to(m)

        # Zoom to view all pins
        m.fit_bounds([(marker_stats['min_lat'], marker_stats['min_lon']), 
                      (marker_stats['max_lat'], marker_stats['max_lon'])])

        # Save final map and use HTMLViewer to view it. Delete html file after window is closed
        temp = tempfile.NamedTemporaryFile(suffix='.html')
        m.save(temp.name)
        app = QApplication(sys.argv)
        viewer = HTMLViewer(temp.name)
        viewer.show()
        app.exec()
        temp.close()

    def _cluster_raw_collection(self) -> None:
        # Get raw documents
        raw_docs : list[dict] = list(self.mongoc[self.store_options["db"]][self.store_options["raw_collection"]].find({}))
        [raw_docs[i].pop('_id') for i in range(len(raw_docs))]
        # from pprint import pprint
        # pprint(raw_docs)
        # print()

        # Compute distance matrix between raw entries
        latlon_tuples = [(doc["lat"], doc["lon"]) for doc in raw_docs]
        dist_matrix = GeoTransforms.distance_matrix_from_latlon(latlon_tuples)
        # print(dist_matrix)
        # print()

        # Cluster entries based on distance matrix
        clustering = AgglomerativeClustering(n_clusters=None,
                                             distance_threshold=self.clustering_options['distance_threshold'],
                                             metric='precomputed',
                                             linkage='single')
        clustering.fit(dist_matrix)

        # Combine measurements belonging to the same cluster into a single document
        out_docs = []
        for label_id in range(max(clustering.labels_)+1):
            docs_in_cluster_idx = np.where(clustering.labels_ == label_id)[0]
            docs_in_cluster = [raw_docs[idx] for idx in docs_in_cluster_idx]
            if len(docs_in_cluster) == 1:
                out_docs.append(docs_in_cluster[0])
            else:
                out_docs.append(self.doc_aggregator.combine(docs_in_cluster))
        
        # Empty clustered measurents database and add freshly combined documents
        self.mongoc[self.store_options["db"]][self.store_options["main_collection"]].drop()
        self.mongoc[self.store_options["db"]][self.store_options["main_collection"]].insert_many(out_docs)

    def _clear(self):
        prompt = input('Are you sure you want to clear all databases? (y/n)\n')
        if prompt.lower() == 'yes' or prompt.lower() == 'y':
            self.mongoc[self.store_options["db"]][self.store_options["raw_collection"]].drop()
            self.mongoc[self.store_options["db"]][self.store_options["main_collection"]].drop()
            print("Databases cleared")
        else:
            print("Databases not cleared")


class GeoTransforms():
    @staticmethod
    def earth_radius_km(lat: float) -> float:
        r1 = 6378.1370  # equatorial radius in km
        r2 = 6356.7523  # polar radius in km
        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        numerator   = (r1 ** 2 * cos_lat) ** 2 + (r2 ** 2 * sin_lat) ** 2
        denominator = (r1 * cos_lat)      ** 2 + (r2 * sin_lat)      ** 2
        radius = math.sqrt(numerator / denominator)
        return radius
    
    @staticmethod
    def distance_from_latlon(tuple1: float, tuple2: float) -> float:
        """
            Returns distance in meters between two coordinates.
            Input tuples should be in (latitude, longitude).
        """
        lat1 = tuple1[0]
        lon1 = tuple1[1]
        lat2 = tuple2[0]
        lon2 = tuple2[1]

        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        radius1 = GeoTransforms.earth_radius_km(lat1)
        radius2 = GeoTransforms.earth_radius_km(lat2)

        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2) ** 2 + \
            math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = ((radius1 + radius2) / 2) * c * 1000  # Convert to meters
        return distance

    @staticmethod
    def distance_matrix_from_latlon(coords: list[tuple]) -> np.ndarray:
        dists = np.zeros((len(coords), len(coords)))
        for i in range(0, len(coords)):
            for j in range(i+1, len(coords)):
                dists[i][j] = GeoTransforms.distance_from_latlon(coords[i], coords[j])
                dists[j][i] = dists[i][j]
        return dists
