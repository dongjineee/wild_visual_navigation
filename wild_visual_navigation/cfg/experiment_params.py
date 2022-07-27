from dataclasses import dataclass, field
from typing import Tuple, Dict, List, Optional
from simple_parsing.helpers import Serializable


@dataclass
class ExperimentParams(Serializable):
    @dataclass
    class GeneralParams:
        name: str = "simple_gcn/debug"
        timestamp: bool = False
        tag_list: List[str] = field(default_factory=lambda: ["debug"])

    general: GeneralParams = GeneralParams()

    @dataclass
    class LoggerParams:
        name: str = "neptune"
        wandb_entity: str = "wild_visual_navigation"
        wandb_project_name: str = "wild_visual_navigation"
        neptune_project_name: str = "ASL/WVN"

    logger: LoggerParams = LoggerParams()

    @dataclass
    class OptimizerParams:
        @dataclass
        class AdamwCfgParams:
            momentum: float = 0.9
            weight_decay: float = 4.0e-05

        name: str = "ADAMW"
        lr: float = 0.01
        adamw_cfg: AdamwCfgParams = AdamwCfgParams()

    optimizer: OptimizerParams = OptimizerParams()

    @dataclass
    class LossParams:
        trav: float = 1.0
        reco: float = 1.0

    loss: LossParams = LossParams()

    @dataclass
    class TrainerParams:
        precision: int = 32
        accumulate_grad_batches: int = 1
        fast_dev_run: bool = False
        limit_train_batches: float = 1.0
        limit_val_batches: float = 1.0
        limit_test_batches: float = 1.0
        max_epochs: int = 1000
        profiler: bool = False
        num_sanity_val_steps: int = 0
        check_val_every_n_epoch: int = 1

    trainer: TrainerParams = TrainerParams()

    @dataclass
    class DataModuleParams:
        visu: bool = True
        batch_size: int = 8
        num_workers: int = 0

    data_module: DataModuleParams = DataModuleParams()

    @dataclass
    class ModelParams:
        name: str = "SimpleGCN"

        @dataclass
        class SimpleGcnCfgParams:
            num_node_features: int = 90
            num_classes: int = 1
            reconstruction: bool = True

        simple_gcn_cfg: SimpleGcnCfgParams = SimpleGcnCfgParams()

    model: ModelParams = ModelParams()

    @dataclass
    class LrMonitorParams:
        logging_interval: str = "step"

    lr_monitor: LrMonitorParams = LrMonitorParams()

    @dataclass
    class CbEarlyStoppingParams:
        active: bool = False

    cb_early_stopping: CbEarlyStoppingParams = CbEarlyStoppingParams()

    @dataclass
    class VisuParams:
        train: int = 2
        val: int = 2
        test: int = 2
        log_every_n_epochs: int = 10

    visu: VisuParams = VisuParams()