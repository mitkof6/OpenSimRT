#
# Resample .trc files using linear interpolation. Creates a copy of the original
# file with the resampled data.
#
# author: Filip Konstantinos <filip.k@ece.upatras.gr>
# %%
import os
import opensim as osim
from utils import read_from_storage
#%%

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'experimental_data')
trc_file = os.path.join(subject_dir, 'experimental_data/task.trc')

#%%

# load and resample markers at 0.01
df = read_from_storage(trc_file)

# create timeseries table
table = osim.TimeSeriesTable()
table.setColumnLabels(df.columns[1:])
table.addTableMetaDataString('DataRate', '100')
table.addTableMetaDataString('Units', 'mm')

# append data to table
for i in range(df.shape[0]):
    table.appendRow(df.time.values[i], osim.RowVector(df.iloc[i, 1:].values))

# write table to file
osim.TRCFileAdapter.write(table.packVec3(),
                          os.path.join(output_dir, 'task_resampled.trc'))

#%%
