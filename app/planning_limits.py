"""Limits shared by map construction, A* and the visibility search."""
from dataclasses import dataclass,fields
import math
import time


class PlanningLimitError(RuntimeError):
    pass


@dataclass(frozen=True)
class PlanningSettings:
    initial_padding: float = 3.0
    expansion_step: float = 3.0
    max_attempts: int = 4
    max_cells: int = 60000
    max_expansions: int = 200000
    timeout: float = 8.0

    def __post_init__(self):
        for field in fields(self):
            value=getattr(self,field.name)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError(f'planning.{field.name} must be finite and positive')
        for name in ('max_attempts','max_cells','max_expansions'):
            if not isinstance(getattr(self,name),int):
                raise ValueError(f'planning.{name} must be an integer')

    @classmethod
    def from_config(cls,config):
        values=config.get('planning',{})
        unknown=set(values)-{f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f'Unknown planning settings: {sorted(unknown)}')
        return cls(**values)


class PlanningBudget:
    def __init__(self,settings):
        self.settings=settings
        self.deadline=time.monotonic()+settings.timeout
        self.expansions=0

    def check(self):
        if time.monotonic()>=self.deadline:
            raise PlanningLimitError(f'Planning time limit reached ({self.settings.timeout}s)')

    def expand(self):
        self.check();self.expansions+=1
        if self.expansions>self.settings.max_expansions:
            raise PlanningLimitError(f'Planning search limit reached ({self.settings.max_expansions} expansions)')

    def cells(self,count):
        self.check()
        if count>self.settings.max_cells:
            raise PlanningLimitError(f'Planning grid limit reached: {count} > {self.settings.max_cells} cells')
