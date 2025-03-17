from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
from cyclonedds.idl.types import sequence


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Sequence(idl.IdlStruct, typename="Sequence"):
    values: sequence[float]
