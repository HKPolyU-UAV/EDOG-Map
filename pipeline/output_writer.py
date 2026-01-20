from __future__ import annotations

import os
import time
import numpy as np
from pystream import Stage
from pipeline.data import AppData


class OutputWriterStage(Stage):
    def __init__(self, save_dir):
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)
        self.qps = []
        self.prev_end_time = time.perf_counter()

        try:
            import matplotlib  
            matplotlib.use("Agg") 
            import matplotlib.pyplot as plt 
            self._plt = plt
            self._has_mpl = True
        except Exception:
            self._plt = None
            self._has_mpl = False

    def __call__(self, data: AppData):
        idx = int(data.metadata.data_idx)
        dgm_pred = data.output_data.dgm_pred

        np.save(os.path.join(self.save_dir, f"{idx:06d}_dgm_pred.npy"), dgm_pred)

        if self._has_mpl and isinstance(dgm_pred, np.ndarray) and dgm_pred.size > 0:
            try:
                self._plt.imsave(
                    os.path.join(self.save_dir, f"{idx:06d}_dgm_pred.png"),
                    dgm_pred
                )
            except Exception:
                pass

        self._update_qps()
        return data

    def _update_qps(self):
        end_time = time.perf_counter()
        self.qps.append(end_time - self.prev_end_time)
        self.prev_end_time = end_time

    def cleanup(self):
        if not self.qps:
            return
        avg = float(np.mean(self.qps[1:]) if len(self.qps) > 1 else np.mean(self.qps))
        if avg > 0:
            print(f"Average throughput {1/avg}")
