from typing import Callable


class DocumentAggregator():
    """
        Class that contains functionality for combining
        dictionaries that correspond to documents inside
        Mongo database.
    """

    def __init__(self, doc_format: dict[str, Callable]):
        """
            Input dict `doc_format` should have the following format:
            {
                "feature1": Callable,
                "feature2": Callable,
                ...
                "featureN": Callable,
            }

            Key corresponds to metric within the documents that
            need to be combined. Value is a Callable that accepts
            a list as argument. This Callable is called when 
            combining the documents' metric.
        """
        if not doc_format:
            raise ValueError("Document format dict can not be empty")
        self.doc_format = doc_format

    def combine(self, docs_to_combine: list[dict]) -> dict:
        out_doc = {}
        for feature, callable in self.doc_format.items():
            out_doc[feature] = callable([doc[feature] for doc in docs_to_combine])
        out_doc['ids'] = DocumentAggregations.join([doc['id'] for doc in docs_to_combine])
        return out_doc


class DocumentAggregations():
    """
        Class containing functions that are responsible for
        combining anomalies found on raw database to a single
        entry in the clustered database
        e.g.
        If we have two raw anomaly entries on the raw database:
        [
            {
                "lat": 40.0,
                "lon": 22.0,
                "address": {...}
            },
            {
                "lat": 40.1,
                "lon": 22.1,
                "address": {...}
            }
        ]
        And these two end up on the same cluster, then you can
        use the functions in this class to combine their metrics 
        into a single entry.
    """
    @staticmethod
    def combine_addresses(address_list: list[dict]) -> dict | list[dict]:
        ret = [address_list[0]]

        for address in address_list[1:]:
            for existing_address in ret:
                if address != existing_address:
                    ret.append(address)
                    break
    
        if len(ret) == 1:
            return ret[0]
        else:
            return ret

    @staticmethod
    def join(lists: list[list]):
        return lists
