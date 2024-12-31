import os

for PATH in ["/opt/homebrew/Cellar/re2/", "/opt/homebrew/Cellar/abseil"]:
    for file_path in [os.path.join(dp, f) for dp, dn, filenames in os.walk(PATH) for f in filenames]:
        print(file_path)
