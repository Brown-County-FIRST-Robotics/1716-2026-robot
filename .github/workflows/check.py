import re
import os


def check():
    pattern = re.compile(r'(//|#)[\s]*(END\s)?T' + 'EMP', re.IGNORECASE)
    # Iterate over the files in the current directory and its subdirectories
    res={}
    for dirpath, dirnames, filenames in os.walk('.'):
        # Exclude the .git directory
        if '.git' in dirnames:
            dirnames.remove('.git')
        if 'build' in dirnames:
            dirnames.remove('build')
        for filename in filenames:
            # Skip files with non-textual extensions
            if not filename.endswith('.py') and not filename.endswith('.java') and not filename.endswith('.gradle'):
                continue
            filepath = os.path.join(dirpath, filename)
            with open(filepath, 'r') as file:
                for line_number, line in enumerate(file, 1):
                    assert not pattern.search(line), "Temp code found on \nfile:"+filepath+"\nLine Number:"+str(line_number)

check()