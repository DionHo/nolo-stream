import pefile
pe = pefile.PE("Nolo_Device.dll")
for exp in pe.DIRECTORY_ENTRY_EXPORT.symbols:
    print(exp.name.decode('utf-8'), hex(exp.address))
