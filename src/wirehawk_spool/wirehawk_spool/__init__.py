"""wirehawk_spool: shared winch/spool model (encoder-counts <-> cable-length)."""

from wirehawk_spool.spool_model import (  # noqa: F401
    WinchSpec,
    length_to_counts,
    counts_to_length,
    lengths_to_counts,
    counts_to_lengths,
    load_spec,
)
