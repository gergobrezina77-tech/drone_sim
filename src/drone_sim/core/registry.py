"""Name -> implementation registry shared by every extension point (vehicles, backends, effects, tasks, ...)."""

from __future__ import annotations


class Registry[T]:
    def __init__(self, kind: str):
        self.kind = kind
        self._items: dict[str, T] = {}

    def register(self, name: str, item: T) -> T:
        if name in self._items:
            raise ValueError(f"{self.kind} {name!r} is already registered")
        self._items[name] = item
        return item

    def get(self, name: str) -> T:
        try:
            return self._items[name]
        except KeyError:
            raise KeyError(f"unknown {self.kind} {name!r}; known: {', '.join(self.names())}") from None

    def names(self) -> list[str]:
        return sorted(self._items)

    def __contains__(self, name: object) -> bool:
        return name in self._items
