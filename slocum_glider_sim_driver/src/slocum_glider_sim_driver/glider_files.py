"""Abstraction over reading and writing files to the glider file
system. Currently reads and writes directly from/to a folder on the local
filesystem, but will likely be extended in the future to handle object storage.

"""

from os.path import exists, join


class FilesystemFile(object):
    def __init__(self, f):
        self.f = f

    def close(self):
        return self.f.close()

    def read(self):
        return self.f.read()

    def write(self, *args, **kwargs):
        return self.f.write(*args, **kwargs)


class FilesystemDriver(object):
    def __init__(self, root_path):
        self.root_path = root_path

    def _full_file_path(self, path):
        if isinstance(path, list):
            return join(self.root_path, *path)
        else:
            return join(self.root_path, path)

    def exists(self, path):
        return exists(self._full_file_path(path))

    def open(self, path, flags='r'):
        return FilesystemFile(open(self._full_file_path(path), flags))
