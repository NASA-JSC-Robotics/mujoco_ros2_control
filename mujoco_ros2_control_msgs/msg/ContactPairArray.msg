# Snapshot of the MuJoCo body pairs currently in contact, as published by the
# ContactPublisherPlugin.

# Time the snapshot was taken. frame_id is unused: the message carries no geometry.
std_msgs/Header header

# One entry per colliding body pair; empty when none of the plugin's participating bodies are
# colliding with each other.
ContactPair[] contact_pairs
