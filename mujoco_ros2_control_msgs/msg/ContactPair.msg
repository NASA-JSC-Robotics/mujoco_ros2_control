# One pair of MuJoCo bodies that are currently in contact, as published by the
# ContactPublisherPlugin. All MuJoCo contacts between the same two bodies are aggregated into a
# single entry, regardless of how many geoms of those bodies touch.

# Names of the two colliding bodies, ordered by ascending body id. A body with no name in the
# MJCF is reported as an empty string; the world body is reported as "world".
string[2] body_names

# Ids of the two colliding bodies, in the same order as body_names.
int32[2] body_ids

# Number of MuJoCo contact points between the two bodies.
int32 num_contacts

# Smallest distance across those contact points, i.e. the deepest penetration.
# Negative values mean the geoms overlap; values up to the contact margin can be positive.
float64 min_distance
