import os

def read_and_delete_first_line(filename):
    first_line = None
    temp_filename = filename + '.tmp'

    with open(filename, 'r') as infile, open(temp_filename, 'w') as outfile:
        first_line = infile.readline().strip()
        for line in infile:
            outfile.write(line)

    # Replace the original file with the updated file
    os.replace(temp_filename, filename)

    return first_line

# Usage
filename = 'example.txt'
first_line = read_and_delete_first_line(filename)
print(f"First line: {first_line}")
