from __future__ import annotations
import os
import json
import bisect
import functools
import numpy as np
from enum import Enum


class AnomalyLabel(Enum):
    NO_ANOMALY = 0
    MANHOLE    = 1
    DEPRESSION = 2
    BUMP       = 3
    CRACK      = 4

    @classmethod
    @functools.cache
    def max_name_len(cls):
        return max([len(name) for name in cls._member_names_])
    
    @property
    def color(self):
        if self.value == 0:
            return [100, 255, 100, 25]
        elif self.value == 1:
            return [220, 200, 120, 25]
        elif self.value == 2:
            return [255, 40, 40, 25]
        elif self.value == 3:
            return [255, 100, 40, 25]
        elif self.value == 4:
            return [255, 180, 40, 25]
        else:
            return [128, 128, 128, 25]


class TransversityLabel(Enum):
    NO_TRANSVERSE = 0
    TRANSVERSE = 1


class SeverityLabel(Enum):
    SMALL_SEVERITY = 0
    MIDDLE_SEVERITY = 1
    HIGH_SEVERITY = 2


class Label():
    def __init__(self, anomaly: AnomalyLabel, 
                       transversity: TransversityLabel, 
                       severity: SeverityLabel):
        self.anomaly = anomaly
        self.transversity = transversity
        self.severity = severity

    def to_dict(self) -> dict:
        return  {"anomaly": {"type": self.anomaly.name, "id": self.anomaly.value},
                 "transversity": {"type": self.transversity.name, "id": self.transversity.value},
                 "severity": {"type": self.severity.name, "id": self.severity.value}}
        
    @classmethod
    def from_dict(cls, in_dict: dict):
        anomaly = AnomalyLabel(in_dict["anomaly"]["id"])
        transversity = TransversityLabel(in_dict["transversity"]["id"])
        severity = SeverityLabel(in_dict["severity"]["id"])

        return cls(anomaly, transversity, severity)


class Annotation():
    def __init__(self, abs_t_start: int,   abs_t_end: int,
                       rel_t_start: float, rel_t_end: float,
                       idx_start: int,     idx_end: int,
                       label: Label):
        self.abs_t_start = abs_t_start
        self.abs_t_end   = abs_t_end
        self.rel_t_start = rel_t_start
        self.rel_t_end   = rel_t_end
        self.idx_start   = idx_start
        self.idx_end     = idx_end
        self.label       = label

    def __lt__(self, other):
        return self.abs_t_start < other.abs_t_start
    
    def __gt__(self, other):
        return self.abs_t_start > other.abs_t_start


class AnnotationList():
    def __init__(self, init_items : list[Annotation] = []):
        self.items = init_items

    def insert(self, ann: Annotation) -> int:
        idx = bisect.bisect_right(self.items, ann)
        self.items.insert(idx, ann)
        return idx

    def remove(self, idx: int) -> Annotation:
        return self.items.pop(idx)
    
    def to_dict(self) -> dict:
        out_dict = {"annotations": []}
        for ann in self.items:
            out_dict["annotations"].append({"label":          ann.label.to_dict(),
                                            "abs_time_start": int(ann.abs_t_start),
                                            "abs_time_end":   int(ann.abs_t_end),
                                            "rel_time_start": float(ann.rel_t_start),
                                            "rel_time_end":   float(ann.rel_t_end),
                                            "idx_start":      int(ann.idx_start),
                                            "idx_end":        int(ann.idx_end)})
        return out_dict
            
    def from_dict(self, in_dict: dict):
        self.items = []
        for entry in in_dict['annotations']:
            self.insert(Annotation(label       = Label.from_dict(entry['label']),
                                   abs_t_start = int(entry['abs_time_start']),
                                   abs_t_end   = int(entry['abs_time_end']),
                                   rel_t_start = float(entry['rel_time_start']),
                                   rel_t_end   = float(entry['rel_time_end']),
                                   idx_start   = int(entry['idx_start']),
                                   idx_end     = int(entry['idx_end'])))

    def to_json(self, path, extra_entries: dict = {}):
        out_dict = extra_entries
        out_dict.update(self.to_dict())
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(out_dict, f, ensure_ascii=False, indent=4)

    def from_json(self, path):
        if os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as f:
                in_dict = json.load(f)
            self.from_dict(in_dict)
        else:
            raise FileNotFoundError(f"Could not find {path}")

    @staticmethod
    def combine(listA: AnnotationList, listB: AnnotationList) -> AnnotationList:
        if not listA.items:
            return listB
        elif not listB.items:
            return listA

        itA = 0
        itB = 0
        out_items = []
        while((itA < len(listA.items)) and (itB < len(listB.items))):
            if listA.items[itA] < listB.items[itB]:
                out_items.append(listA.items[itA])
                itA += 1
            else:
                out_items.append(listB.items[itB])
                itB += 1

        if itA == len(listA.items):
            out_items.extend(listB.items[itB:])
        else:
            out_items.extend(listA.items[itA:])

        return AnnotationList(out_items)

    def to_time_samples(self, rel_time: np.ndarray) -> list[list[Label]]:
        """
            Returns a list of lists of 'Label' instances.

            The top-level list has the same length as 'rel_time' 
            argument, and each element indicates which labels are
            present in the corresponding time point in 'rel_time'
            vector.

            The inner lists contain the labels that this data point
            has (could be more than one or no label at all).

            e.g.
            >>> rel_time = np.arange(0.0, 4.0, 0.1) # len = 40

            >>> ann1 = Annotation(rel_t_start=1, rel_t_end=1.5, label=label1, ...)
            >>> ann2 = Annotation(rel_t_start=2, rel_t_end=2.5, label=label2, ...)         
            >>> ann_list = AnnotationList([ann1, ann2])

            >>> ann_list.to_time_samples(rel_time)
                [[], ..., [label1], ..., [label1], ..., [], ..., [label2], ..., [label2], ..., [], ...]
                             ^              ^                       ^              ^
                        index where    index where             index where    index where
                        rel_time==1.0  rel_time==1.5           rel_time==2.0  rel_time==2.5
        """
        ret : list[list[Label]] = [[] for _ in range(len(rel_time))]
        for ann in self.items:
            idx = np.where(np.logical_and(rel_time >= ann.rel_t_start, rel_time <= ann.rel_t_end))[0]
            [ret[i].append(ann.label) for i in idx]
        return ret

    def to_time_range_indices(self, rel_time: np.ndarray) -> list[tuple[Label, int, int]]:
        """
            Returns a list of tuples, with each tuple containing
            one `Label` instance and two `int`s.

            Each tuple corresponds to an annotation. The `Label`
            instance contains info regarding the lable itself, 
            while the two ints signify the index range relative 
            to `rel_time` argument where the `Label` applies
            (inclusive).
        """
        ret : list[tuple[Label, int, int]] = [None for _ in range(len(self.items))]
        for i, ann in enumerate(self.items):
            start_idx = np.searchsorted(rel_time, ann.rel_t_start, 'left')
            end_idx = np.searchsorted(rel_time, ann.rel_t_end, 'right')
            ret[i] = (ann.label, start_idx, end_idx-1)
        return ret




if __name__ == '__main__':
    annotation1 = Annotation(abs_t_start=0, abs_t_end=2,
                             rel_t_start=1, rel_t_end=1.5,
                             idx_start=0, idx_end=2, label=Label(anomaly=AnomalyLabel.BUMP,
                                                                 transversity=TransversityLabel.NO_TRANSVERSE,
                                                                 severity=SeverityLabel.MIDDLE_SEVERITY))
    
    annotation2 = Annotation(abs_t_start=1, abs_t_end=3,
                             rel_t_start=2.5, rel_t_end=3,
                             idx_start=0, idx_end=2, label=Label(anomaly=AnomalyLabel.DEPRESSION,
                                                                 transversity=TransversityLabel.NO_TRANSVERSE,
                                                                 severity=SeverityLabel.HIGH_SEVERITY))
    
    annotation3 = Annotation(abs_t_start=2, abs_t_end=4,
                             rel_t_start=2.7, rel_t_end=3.2,
                             idx_start=0, idx_end=2, label=Label(anomaly=AnomalyLabel.CRACK,
                                                                 transversity=TransversityLabel.NO_TRANSVERSE,
                                                                 severity=SeverityLabel.SMALL_SEVERITY))
    
    ann_list = AnnotationList()
    ann_list.insert(annotation1)
    ann_list.insert(annotation2)
    ann_list.insert(annotation3)

    rel_time = np.arange(0.0, 4.0, 0.1)

    # labels = ann_list.to_time_samples(rel_time)
    # for i, annotations in enumerate(labels):
    #     label_names = [ann.anomaly.name for ann in annotations]
    #     print(f'{rel_time[i]:.1f}: {label_names}')

    print(rel_time)
    labels_ranges = ann_list.to_time_range_indices(rel_time)
    for i, ran in enumerate(labels_ranges):
        print(f'ann_list[{i}]: {ran[1]} - {ran[2]}')
