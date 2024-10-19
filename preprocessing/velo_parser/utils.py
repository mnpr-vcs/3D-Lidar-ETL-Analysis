import numpy as np

def read_uint32(data, idx):
  return data[idx] + data[idx+1]*256 + data[idx+2]*256*256 + data[idx+3]*256*256*256


def write_txt(path, timestamps, X, Y, Z, intensities=None):
    header = "X,Y,Z,intensity\n"
    try:
        fp = open(path, 'w')
        fp.write(header)
    except Exception as ex:
        print(str(ex))
        return

    M = np.vstack(( X, Y, Z))

    if intensities is not None:
        M = np.vstack((M, intensities))

    np.savetxt(fp, M.T, fmt=('%.6f', '%.6f', '%.6f', '%d'), delimiter=',')
    fp.close()


def write_pcd(path, X, Y, Z,  intensities=None):
    template = """VERSION {}\nFIELDS {}\nSIZE {}\nTYPE {}\nCOUNT {}\nWIDTH {}\nHEIGHT {}\nVIEWPOINT {}\nPOINTS {}\nDATA {}\n"""

    X = X.astype(np.float32).reshape(1, -1)
    Y = Y.astype(np.float32).reshape(1, -1)
    Z = Z.astype(np.float32).reshape(1, -1)

    if intensities is not None:
        I = intensities.astype(np.float32).reshape(1, -1)
        M = np.hstack((X.T, Y.T, Z.T, I.T))
        pc_data = M.view(np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('i', np.float32)]))
        tmpl = template.format("0.7", "x y z intensity", "4 4 4 4", "F F F F", "1 1 1 1", pc_data.size, "1",
                               "0 0 0 1 0 0 0", pc_data.size, "binary")
    else:
        M = np.hstack((X.T, Y.T, Z.T))
        pc_data = M.view(np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)]))
        tmpl = template.format("0.7", "x y z", "4 4 4", "F F F", "1 1 1", pc_data.size, "1", "0 0 0 1 0 0 0",
                               pc_data.size, "binary")

    fp = open(path, 'wb')
    fp.write(tmpl.encode())
    fp.write(pc_data.tostring())
    fp.close()