"""Read-only model/profile and offline evidence qualification helpers.

The qualification package is intentionally independent from ROS, OpenVINO,
camera SDKs and serial hardware.  It audits the versioned YAML contracts and
reuses :mod:`tools.offline_evidence_report` for CSV/evidence-bundle checks.
"""

from .model_profile_audit import (
    AuditResult,
    Finding,
    audit_model_profile,
    audit_pnp_config,
)
from .offline_qualification import qualify_offline, status_code

__all__ = [
    "AuditResult",
    "Finding",
    "audit_model_profile",
    "audit_pnp_config",
    "qualify_offline",
    "status_code",
]
