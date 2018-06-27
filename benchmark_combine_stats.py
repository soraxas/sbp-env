import sys, os, glob
import re
import csv

import rrtstar
import subprocess
import openpyxl
from openpyxl import Workbook
from openpyxl.chart import (
    ScatterChart,
    Reference,
    Series,
)
from openpyxl.chart.error_bar import ErrorBars
from openpyxl.chart import data_source
from openpyxl.utils import get_column_letter

STATS_FILE_NAME = 'stats.xlsx'

INFO_MAX_COL = 6  # amount of column for info section
INFO_EXTRA_COL = 4  # column extra in info section
INFO_END_ROW = 2  # row number that info end
DATA_BEGIN_ROW = 4  # row number that data begin
DATA_COST_COL = 6  # column number of cost
DATA_MAX_COL = 6  # amount of column for data section
DATA_NUMNODE_COL = 1  # column number of number of nodes

def duplicate_col(start, end, ws, row, formatted_str, datas_rotates):
    # convert to 0-based system
    start -= 1
    end -= 1 # end is assume to not be inclusive
    _count = 0
    for column in range(start, (end-start)*2+start):
        column += 1  # convert to 1-based system
        column_letter = get_column_letter(int((1+column-start)/2) + start)
        # switch between two types on each iterations
        data = datas_rotates[_count]
        _count += 1
        if _count >= len(datas_rotates):
            _count = 0
        val = formatted_str.format(c=column_letter, r=row, data=data)
        ws.cell(row=row, column=column, value=val)

def combine_result(folder):

    def save_csv_as_sheet(policy):
        # create a list to store the newly created sheets
        sheets = []
        for filename in glob.glob("policy={}_*.csv".format(policy)):
            timestamp = re.search('.*timestamp=([0-9]+-[0-9]+).csv', filename).group(1)
            newsheet = "{}_{}".format(policy, timestamp)
            ws = wb.create_sheet(newsheet)
            sheets.append(newsheet)
            with open(filename) as f:
                reader = csv.reader(f)
                for row_index, row in enumerate(reader):
                    row_index += 1 # conver to 1-based system
                    if row_index > DATA_BEGIN_ROW:
                        row = (float(x) if x != 'inf' else x for x in row)
                    ws.append(row)
        return sheets, ws.max_row

    def create_main_stats():
        ############################################################
        ##              Create Scatter Chart at main              ##
        ############################################################
        ws = wb.create_sheet('main', index=0)
        start, end = 2, DATA_MAX_COL+1
        for i in range(start, end):
            plotting_col = (i-start)*2 + start

            chart = ScatterChart()
            chart.title = wb[sheets[0]].cell(row=DATA_BEGIN_ROW, column=i).value
            # chart.style = 13
            chart.x_axis.title = "Number of Nodes"
            chart.y_axis.title = wb[sheets[0]].cell(row=DATA_BEGIN_ROW, column=i).value
            xvalues = Reference(wb[policies[0]], min_col=1, min_row=DATA_BEGIN_ROW, max_row=wb[policies[0]].max_row)
            for j, policy in enumerate(policies):
                stat_ws = wb[policy]
                values = Reference(stat_ws, min_col=plotting_col, min_row=DATA_BEGIN_ROW, max_row=stat_ws.max_row)
                stdev = Reference(stat_ws, min_col=plotting_col+1, min_row=DATA_BEGIN_ROW+1, max_row=stat_ws.max_row)

                # convert reference to data source
                stdev_data_source = data_source.NumDataSource(numRef=data_source.NumRef(f=stdev))
                error_bars = ErrorBars(errDir='y', errValType='cust', plus=stdev_data_source, minus=stdev_data_source)

                series = Series(values, xvalues, title=policy)
                series.errBars = error_bars
                chart.series.append(series)
                ############################################################
                ##   Store simple stats at main screen for easy viewing   ##
                ############################################################
                ws.cell(row=2+j, column=1, value=policy)
                for local_col_idx, col in enumerate(range(INFO_MAX_COL + 1, INFO_MAX_COL + INFO_EXTRA_COL*2 + 1)):
                    # Set titles
                    val = "='{sheet}'!{c}{r}".format(sheet=policy,
                                                    c=get_column_letter(col),
                                                    r=1)
                    ws.cell(row=1, column=local_col_idx+2, value=val)
                    # Set values
                    val = "='{sheet}'!{c}{r}".format(sheet=policy,
                                                    c=get_column_letter(col),
                                                    r=2)
                    ws.cell(row=2+j, column=local_col_idx+2, value=val)

            ws.add_chart(chart, 'A'+str(4+15*(i-start)))


    os.chdir(folder)
    if os.path.isfile(STATS_FILE_NAME):
        # skip as this had been processed
        # return
        pass
    print(folder)
    wb = Workbook()
    ws = wb.active
    # remove default sheet
    wb.remove(ws)

    policies = ['random', 'disjoint']
    for idx, policy in enumerate(policies):
        sheets, num_rows = save_csv_as_sheet(policy)
        ws = wb.create_sheet(policy, index=idx)
        create_stats_view(ws, sheets, num_rows=num_rows)
        create_stats_time_taken(wb, raw_sheets=sheets, stat_ws=ws)

    create_main_stats()

        # wb.save("scatter.xlsx")


    wb.save(STATS_FILE_NAME)
    # delete other csv
    # for filename in glob.glob("*.csv"):
    #     os.remove(filename)


def combine_all_results(maindir):
    all_dirs = (os.path.join(maindir, x) for x in os.listdir(maindir))
    all_dirs = [x for x in all_dirs if os.path.isdir(x)]
    print('===== Combining csv to as xlxs sheet =====')
    for subdir in all_dirs:
        combine_result(subdir)
        os.chdir(maindir) # switch back to main folder

############################################################
##                    HELPER FUNCTIONS                    ##
############################################################

def create_stats_time_taken(wb, raw_sheets, stat_ws):
    """Create stats of the number of node (and time) when the goal node is first found"""
    def find_time_taken(ws):
        solution_found = None
        terminate = None
        for r in range(DATA_BEGIN_ROW + 1, ws.max_row + 1):
            if solution_found is None and ws.cell(row=r, column=DATA_COST_COL).value != 'inf':
                # first time the policy found a solution
                # Index 1 as node count, index 2 as node cost
                solution_found = (ws.cell(row=r, column=DATA_NUMNODE_COL).value,
                                  ws.cell(row=r, column=DATA_COST_COL).value)
        # finish! lets check the final value
        if ws.cell(row=r, column=DATA_COST_COL).value != 'inf':
            terminate = (ws.cell(row=r, column=DATA_NUMNODE_COL).value,
                         ws.cell(row=r, column=DATA_COST_COL).value)
        return solution_found, terminate

    # find all the solution stats
    for sheetname in raw_sheets:
        ws = wb[sheetname]
        solution_found, terminate = find_time_taken(ws)
        if solution_found is None:
            solution_found = (99999999, 99999999)
        if terminate is None:
            terminate = (99999999, 99999999)
        ws.cell(row=1, column=INFO_MAX_COL+1, value='SoluFoundAt')
        ws.cell(row=2, column=INFO_MAX_COL+1, value=solution_found[0])
        ws.cell(row=1, column=INFO_MAX_COL+2, value='SoluFoundAtCost')
        ws.cell(row=2, column=INFO_MAX_COL+2, value=solution_found[1])
        ws.cell(row=1, column=INFO_MAX_COL+3, value='TermAt')
        ws.cell(row=2, column=INFO_MAX_COL+3, value=terminate[0])
        ws.cell(row=1, column=INFO_MAX_COL+4, value='TermCost')
        ws.cell(row=2, column=INFO_MAX_COL+4, value=terminate[1])
    # create stats at main page
    duplicate_col(start=INFO_MAX_COL+1,
                  end=INFO_MAX_COL+5,
                  ws=stat_ws,
                  row=1,
                  formatted_str="='{sh}'!{{c}}{{r}}&\"{{data}}\"".format(sh=raw_sheets[0]),
                  datas_rotates=('(mean)', '(stdev)'))
    duplicate_col(start=INFO_MAX_COL+1,
                  end=INFO_MAX_COL+5,
                  ws=stat_ws,
                  row=2,
                  formatted_str="={{data}}('{s_begin}:{s_end}'!{{c}}{{r}})".format(
                      s_begin=raw_sheets[0],
                      s_end=raw_sheets[-1]),
                  datas_rotates=('AVERAGE','STDEV'))

def create_stats_view(main_ws, sheets, num_rows):
    ############################################################
    ##             first 4 rows just mimic others             ##
    ############################################################
    for column in range(1, INFO_MAX_COL + 1):
        column_letter = get_column_letter(column)
        for row in range(1, INFO_END_ROW + 1):
            val = "='{sheet}'!{c}{r}".format(sheet=sheets[0],
                                            c=column_letter,
                                            r=row)
            main_ws.cell(row=row, column=column, value=val)
            # main_ws[column_letter + str(row)] =
    ############################################################
    ##    The reset of the rows should be average and stdev   ##
    ############################################################
    ## Title row
    row = 4
    # First title in column A:
    val = "='{sheet}'!{c}{r}".format(sheet=sheets[0],
                                    c='A',r=row)
    main_ws.cell(row=row, column=1, value=val)
    # others

    duplicate_col(start=2,
                  end=DATA_MAX_COL+1,
                  ws=main_ws,
                  row=row,
                  formatted_str="='{sh}'!{{c}}{{r}}&\"{{data}}\"".format(sh=sheets[0]),
                  datas_rotates=('(mean)', '(stdev)'))
    ############################################################
    ## Values
    for row in range(DATA_BEGIN_ROW + 1, num_rows + 1):
        val = "='{sheet}'!{c}{r}".format(sheet=sheets[0],
                                            c='A',r=row)
        main_ws.cell(row=row, column=1, value=val)
        # make this lag behind as we need double the amount of original columns number
        duplicate_col(start=2,
                      end=DATA_MAX_COL+1,
                      ws=main_ws,
                      row=row,
                      formatted_str="={{data}}('{s_begin}:{s_end}'!{{c}}{{r}})".format(
                          s_begin=sheets[0],
                          s_end=sheets[-1]),
                      datas_rotates=('AVERAGE','STDEV'))

if __name__ == '__main__':
    dirname = os.path.abspath('benchmark_copy')
    combine_all_results(dirname)
