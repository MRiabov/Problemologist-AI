content = open('worker/simulation/genesis_backend.py').read().split('\n')
for i in range(len(content)):
    line = content[i]
    if 'vels = entity.get_dofs_velocity().cpu().numpy()' in line and 'idx' in content[i+3]:
        # found the line, let's fix indentation for the whole block
        # line 873 in original cat -n was vels = ...
        pass
# Actually I will just rewrite the whole file with correct indentation manually in the cat <<EOF.
