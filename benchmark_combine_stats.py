import sys, os, glob
import re
import csv

import rrtstar
import subprocess
import openpyxl
from openpyxl import Workbook
from openpyxl.utils import get_column_letter

STATS_FILE_NAME = 'stats.xlsx'

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
                    if row_index >= 4:
                        row = (float(x) if x != 'inf' else x for x in row)
                    ws.append(row)
        return sheets, ws.max_row

    def create_stats_view(main_ws, sheets, num_rows):
        ############################################################
        ##             first 4 rows just mimic others             ##
        ############################################################
        for column in range(1,7):
            column_letter = get_column_letter(column)
            for row in (1, 2):
                val = "='{sheet}'!{c}{r}".format(sheet=sheets[0],
                                                c=column_letter,
                                                r=row)
                main_ws.cell(row=row, column=column, value=val)
                # main_ws[column_letter + str(row)] =
        ############################################################
        ##    The reset of the rows should be average and stdev   ##
        ############################################################
        ## Title
        row = 4
        # First title
        val = "='{sheet}'!{c}{r}".format(sheet=sheets[0],
                                        c='A',r=row)
        main_ws.cell(row=row, column=1, value=val)
        # others
        column_on_other_page = 2
        for column in range(2, 12):
            # make this lag behind as we need double the amount of original columns number
            column_letter = get_column_letter(int(column/2) + 1)
            # switch between two types on each iterations
            if column % 2 == 0:
                datatype = '(mean)'
            else:
                datatype = '(stdev)'
            val = "='{sheet}'!{c}{r}&\"{datatype}\"" .format(sheet=sheets[0],
                                                    c=column_letter,
                                                    r=row,
                                                    datatype=datatype)
            main_ws.cell(row=row, column=column, value=val)
        ############################################################
        ## Values
        for row in range(5, num_rows + 1):
            val = "='{sheet}'!{c}{r}".format(sheet=sheets[0],
                                                c='A',r=row)
            main_ws.cell(row=row, column=1, value=val)
            # make this lag behind as we need double the amount of original columns number
            for column in range(2, 12):
                column_letter = get_column_letter(int(column/2) + 1)
                if column % 2 == 0:
                    func = 'AVERAGE'
                else:
                    func = 'STDEV.S'
                val = "={func}('{s_begin}:{s_end}'!{c}{r})" .format(s_begin=sheets[0],
                                                                   s_end=sheets[-1],
                                                                   c=column_letter,
                                                                   r=row,
                                                                   func=func)
                main_ws.cell(row=row, column=column, value=val)
        # main_ws.append([""])

    os.chdir(folder)
    if os.path.isfile(STATS_FILE_NAME):
        # skip as this had been processed
        return
    print('===============')
    print(folder)
    wb = Workbook()
    ws = wb.active
    # remove default sheet
    wb.remove(ws)

    random = 'random'
    disjoint = 'disjoint'

    sheets, num_rows = save_csv_as_sheet(random)
    ws = wb.create_sheet(random, index=0)
    create_stats_view(ws, sheets, num_rows=num_rows)

    sheets, num_rows = save_csv_as_sheet(disjoint)
    ws = wb.create_sheet(disjoint, index=1)
    create_stats_view(ws, sheets, num_rows=num_rows)

    wb.save(STATS_FILE_NAME)
    # delete other csv
    for filename in glob.glob("*.csv"):
        os.remove(filename)


def combine_all_results(maindir):
    all_dirs = (os.path.join(maindir, x) for x in os.listdir(maindir))
    all_dirs = [x for x in all_dirs if os.path.isdir(x)]
    for subdir in all_dirs:
        combine_result(subdir)
        os.chdir(maindir) # switch back to main folder

def create_stats_of_goal_first_found(maindir):
    """Create stats of the number of node (and time) when the goal node is first found"""
    maindir = os.path.abspath(maindir)
    all_dirs = (os.path.join(maindir, x) for x in os.listdir(maindir))
    all_dirs = [x for x in all_dirs if os.path.isdir(x)]
    for subdir in all_dirs:
        wb = openpyxl.load_workbook(os.path.join(subdir, STATS_FILE_NAME))
        print(wb.sheetnames)


if __name__ == '__main__':
    dirname = os.path.abspath('benchmark_copy')
    combine_all_results(dirname)
    os.chdir(dirname)
    create_stats_of_goal_first_found(dirname)
