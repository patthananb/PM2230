import xlrd
wb = xlrd.open_workbook('PM2230 Power Meter Readings/Public_PM2xxx_PMC Register List_v1001.xls')
ws = wb.sheet_by_name('Register List')

keywords = ['Current', 'Voltage', 'Power', 'Frequency', 'Factor']
print('Row\tCategory\tSubCat1\tSubCat2\tDescription\tPM2230\tRegister\tUnits\tSize\tDataType\tAccess')
for r in range(ws.nrows):
    desc = str(ws.cell_value(r, 3))
    cat = str(ws.cell_value(r, 0))
    subcat1 = str(ws.cell_value(r, 1))
    subcat2 = str(ws.cell_value(r, 2))
    pm2230 = str(ws.cell_value(r, 8))
    register = str(ws.cell_value(r, 10))
    units = str(ws.cell_value(r, 11))
    size = str(ws.cell_value(r, 12))
    dtype = str(ws.cell_value(r, 14))
    access = str(ws.cell_value(r, 15))
    if any(kw.lower() in desc.lower() or kw.lower() in subcat1.lower() or kw.lower() in subcat2.lower() for kw in keywords):
        if 'Y' in pm2230:
            print(f'{r}\t{cat}\t{subcat1}\t{subcat2}\t{desc}\t{pm2230}\t{register}\t{units}\t{size}\t{dtype}\t{access}')
