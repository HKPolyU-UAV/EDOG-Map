from __future__ import annotations

import numpy as np
from pystream import Stage
from pipeline.data import AppData


def _is_empty_ndarray(x) -> bool:
    return x is None or (isinstance(x, np.ndarray) and x.size == 0)


class RoadSegmentationStage(Stage):

    def __init__(self, model_path, input_size):
        self.model_path = model_path
        self.input_size = int(input_size)
        self._model = None

    def __call__(self, data: AppData):
        if _is_empty_ndarray(data.input_data.input_image):
            data.output_data.segmentation_pred = np.array([])
            return data

        if self._model is None:
            self._model = RoadSegmentation(self.model_path, self.input_size)

        data.output_data.segmentation_pred = self._model.infer(data.input_data.input_image)
        return data

    def cleanup(self):
        pass


class RoadSegmentation:
    def __init__(self, model_path, input_size):
        import tensorflow as tf 
        from libs.sensors import segmentation_inference 

        self._segmentation_inference = segmentation_inference
        self.input_size = int(input_size)

        with open(model_path, "rb") as f:
            graph_def = tf.compat.v1.GraphDef.FromString(f.read()) 
        graph = tf.Graph()
        with graph.as_default(): 
            tf.import_graph_def(graph_def=graph_def, name="")
        self.infer_sess = tf.compat.v1.Session(graph=graph)

        self._segmentation_inference(
            np.ones((600, 600, 3), dtype=np.uint8),
            self.infer_sess,
            self.input_size,
            0.5,
        )

    def infer(self, input_image):
        _, segm_pred = self._segmentation_inference(
            input_image, self.infer_sess, self.input_size, 0.5
        )
        return segm_pred
