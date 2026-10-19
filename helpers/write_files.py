from helpers.config import FILE_NAME

def write_to_file(data, filename=FILE_NAME):
    """Write the given data to a file with the specified filename."""
    with open(filename, 'w') as f:
        f.write(data)