"""Aggregate topic health into configurable information-domain decisions."""
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from odd_extraction.validation.topic_health import TopicHealth

GOOD_TOPIC_STATES = {"usable", "degraded", "metadata_only"}


@dataclass
class DomainHealth:
    name: str
    health: str
    usable_topics: List[str] = field(default_factory=list)
    degraded_topics: List[str] = field(default_factory=list)
    unusable_topics: List[str] = field(default_factory=list)
    missing_topics: List[str] = field(default_factory=list)
    insufficient_data_topics: List[str] = field(default_factory=list)
    reasons: List[str] = field(default_factory=list)


def evaluate_domains(topic_health: Dict[str, TopicHealth], config: dict) -> Dict[str, DomainHealth]:
    domains = {}
    for name, domain_cfg in (config.get("domains") or {}).items():
        topics = domain_cfg.get("topics") or []
        policy = domain_cfg.get("policy", "any")
        minimum = int(domain_cfg.get("minimum_usable_topics", len(topics) if policy == "all" else 1))
        usable: List[str] = []
        degraded: List[str] = []
        missing: List[str] = []
        insufficient: List[str] = []
        unusable: List[str] = []
        for topic in topics:
            state = topic_health.get(topic)
            if state is None or state.health == "missing":
                missing.append(topic)
            elif state.health in ("usable", "metadata_only"):
                usable.append(topic)
            elif state.health == "degraded":
                degraded.append(topic)
            elif state.health == "insufficient_data":
                insufficient.append(topic)
            else:
                unusable.append(topic)
        suitable_count = len(usable) + len(degraded)
        if policy == "all":
            health = "usable" if len(usable) == len(topics) else ("degraded" if suitable_count == len(topics) else "unusable")
        else:
            if suitable_count >= minimum:
                health = "usable" if len(usable) >= minimum else "degraded"
            else:
                health = "unusable"
        reasons = []
        if missing:
            reasons.append("missing_topics")
        if degraded:
            reasons.append("degraded_topics")
        if insufficient:
            reasons.append("insufficient_data_topics")
        if unusable:
            reasons.append("unusable_topics")
        domains[name] = DomainHealth(name, health, usable, degraded, unusable, missing, insufficient, reasons)
    return domains


def evaluate_analysis(domains: Dict[str, DomainHealth], config: dict) -> Tuple[str, List[str], Dict[str, object]]:
    analysis_cfg = config.get("analysis") or {}
    core = analysis_cfg.get("core_domains") or ["ego_motion", "localization"]
    usable_core = [d for d in core if domains.get(d) and domains[d].health in ("usable", "degraded")]
    if not usable_core:
        status = "rejected"
        reasons = ["no_usable_core_domain"]
    else:
        full_requires = analysis_cfg.get("full_requires") or list(domains.keys())
        bad = [d for d in full_requires if not domains.get(d) or domains[d].health != "usable"]
        status = "full" if not bad else "partial"
        reasons = ["%s_%s" % (d, domains[d].health if d in domains else "missing") for d in bad]
    supported = []
    unsupported = []
    for output, cfg in (analysis_cfg.get("outputs") or {}).items():
        reqs = cfg.get("requires") or []
        if all(domains.get(d) and domains[d].health in ("usable", "degraded") for d in reqs):
            supported.append(output)
        else:
            unsupported.append(output)
    return status, reasons, {"recommended": status != "rejected", "supported_outputs": supported, "unsupported_or_degraded_outputs": unsupported}
