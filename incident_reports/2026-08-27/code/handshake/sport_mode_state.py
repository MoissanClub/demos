"""CycloneDDS binding for the humanoid SportModeState omitted by SDK Python."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class SportModeState_(idl.IdlStruct, typename="unitree_hg.msg.dds_.SportModeState_"):
    fsm_id: types.uint32
    fsm_mode: types.uint32
    task_id: types.uint32
    task_time: types.float32
