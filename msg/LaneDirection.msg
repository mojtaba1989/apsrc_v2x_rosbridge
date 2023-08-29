bool ingressPath
bool egressPath

# -- With bits as defined:
# -- Allowed directions of travel in the lane object
# -- All lanes are described from the stop line outwards
# ingressPath
# (0),
# -- travel from rear of path to front
# -- is allowed
# egressPath
# (1)
# -- travel from front of path to rear
# -- is allowed
# -- Notes: No Travel, i.e. the lane object type does not support
# --
# travel (medians, curbs, etc.) is indicated by not
# --
# asserting any bit value
# --
# Bi-Directional Travel (such as a ped crosswalk) is
# --
# indicated by asserting both of the bits
# } (SIZE (2))