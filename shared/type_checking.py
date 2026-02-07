from beartype import beartype

# Centralized type checking decorator
# We export this so we can easily swap implementations or disable it globally if needed
type_check = beartype
