from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from shared.cots.base import COTSPart
from shared.cots.parts.motors import ServoMotor
from shared.cots.runtime import get_catalog_item_by_part_id


@dataclass(frozen=True, slots=True)
class GeometryInterfaceContract:
    """Typed contract describing the public geometry guarantees of a provider."""

    canonical_local_frame: str
    required_features: tuple[str, ...]
    notes: str = ""


@dataclass(frozen=True, slots=True)
class GeometryProvider:
    """Deterministic geometry adapter for one catalog-backed COTS part."""

    catalog_part_id: str
    family: str
    variant: str
    cots_class: type[COTSPart]
    factory_kwargs: tuple[tuple[str, Any], ...]
    interface_contract: GeometryInterfaceContract
    catalog_aliases: tuple[str, ...] = ()

    def instantiate(self, *, label: str | None = None) -> COTSPart:
        """Instantiate the concrete COTS geometry object for this provider."""
        kwargs = dict(self.factory_kwargs)
        if label is not None:
            return self.cots_class(label=label, **kwargs)
        return self.cots_class(**kwargs)


def _build_motor_interface_contract() -> GeometryInterfaceContract:
    return GeometryInterfaceContract(
        canonical_local_frame=(
            "origin at mounting datum; local +Z aligns with the shaft axis"
        ),
        required_features=(
            "outer envelope",
            "shaft axis",
            "mounting datum",
            "mounting footprint",
            "connector clearance",
        ),
        notes=(
            "Interface-faithful proxy only; vendor mesh fidelity is intentionally out "
            "of scope for the MVP."
        ),
    )


def _build_motor_provider(variant: str) -> GeometryProvider:
    catalog_part_id = f"ServoMotor_{variant}"
    return GeometryProvider(
        catalog_part_id=catalog_part_id,
        family="motor",
        variant=variant,
        cots_class=ServoMotor,
        factory_kwargs=(("size", catalog_part_id),),
        interface_contract=_build_motor_interface_contract(),
    )


SUPPORTED_GEOMETRY_PROVIDERS: tuple[GeometryProvider, ...] = tuple(
    _build_motor_provider(variant) for variant in ServoMotor.motor_data.keys()
)

_GEOMETRY_PROVIDER_BY_PART_ID = {
    provider.catalog_part_id: provider for provider in SUPPORTED_GEOMETRY_PROVIDERS
}


def resolve_cots_provider(part_id: str) -> GeometryProvider:
    """Resolve a catalog-backed `part_id` to a deterministic geometry provider."""
    normalized = str(part_id).strip()
    if not normalized:
        raise ValueError("part_id must be a non-empty string")

    catalog_item = get_catalog_item_by_part_id(normalized)
    if catalog_item is None:
        raise ValueError(f"Unknown catalog part_id '{part_id}'")

    provider = _GEOMETRY_PROVIDER_BY_PART_ID.get(catalog_item.part_id)
    if provider is None:
        raise ValueError(
            f"No geometry provider registered for catalog part_id '{catalog_item.part_id}'"
        )
    return provider


def supported_cots_geometry_hints() -> tuple[str, ...]:
    """Return commented starter-hint lines derived from the provider registry."""
    hints: list[str] = []
    if not SUPPORTED_GEOMETRY_PROVIDERS:
        return tuple(hints)

    hints.append("# COTS import hint:")
    provider = SUPPORTED_GEOMETRY_PROVIDERS[0]
    if provider.family == "motor":
        class_name = provider.cots_class.__name__
        hints.append(f"# from shared.cots.parts.motors import {class_name}")
        hints.append(
            f'# motor = {class_name}.from_catalog_id("{provider.catalog_part_id}")'
        )
    return tuple(hints)
