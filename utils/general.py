# coding=utf-8

import sys
from PySide6.QtWidgets import QFileDialog, QListView, QAbstractItemView, QTreeView, QTableView
import re
from nptdms import TdmsWriter, ChannelObject
import numpy as np
from nptdms.types import TimeStamp
import os

def find_dir_path(dir_name):
    """
    作用：查看包含folder_name的模块路径，并返回路径
    """
    # folder_name = 'renamedata'
    folder_name = dir_name
    for path in sys.path:
        # print(path)
        if folder_name in path:
            return path
    return None

def getFoldersPath(m_directory):
    """
    作用：选择多个文件夹的对话框，并获取这些文件夹的路径。
    """
    fileDialog = QFileDialog()
    fileDialog.setDirectory(m_directory)
    fileDialog.setFileMode(QFileDialog.FileMode.Directory)
    fileDialog.setOption(QFileDialog.Option.DontUseNativeDialog, True)
    listView: QListView = fileDialog.findChild(QListView, "listView")
    if listView:
        listView.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)

    treeView:QTreeView = fileDialog.findChild(QTreeView, "treeView")
    if treeView:
        treeView.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
    
    if fileDialog.exec_():
        folders = fileDialog.selectedFiles()
        return folders
    return None

def extract_numbers_from_string(string):
    pattern = r'\d+$'  # 匹配字符串末尾的数字
    match = re.search(pattern, string)
    if match:
        return match.group()
    else:
        return None
    


def get_listview_items(listview: QListView):
        model: QListView = listview.model()
        item_list = []
        for row in range(model.rowCount()):
            index = model.index(row, 0)
            item = model.data(index, 0)
            item_list.append(item)
        return item_list


def find_all_diff_indices(arr, threshold=1):
    """
    Python找出所有数组中后一个减前一个等于1的元素索引，然后封装成函数
    """
    diff_indices = []
    for i in range(1, len(arr)):
        if arr[i] - arr[i-1] == threshold: # 上升沿
            diff_indices.append(i) # 在这里找到对应的索引，从而找到时间点
    return diff_indices

def find_first_last_indices(arr, threshold=1):
    """
    截取数据段
    """
    diff_indices = []
    for i in range(1, len(arr)):
        if arr[i] > threshold: # 大于阈值的第一个数
            diff_indices.append(i)
            break
    for i in range(len(arr)-1, -1, -1):
        if arr[i] > threshold: # 大于阈值的倒数第一个数
            diff_indices.append(i)
            break
    return diff_indices

# 读取txt文件
def read_large_file(file_path):
    with open(file_path, 'r') as file:
        for line in file:
            yield line


def rename_duplicates(my_list):
    """重命名重复元素"""
    counts = {}
    new_list = []

    for item in my_list:
        if item not in counts:
            counts[item] = 1
            new_list.append(item)
        else:
            counts[item] += 1
            new_list.append(f"{item}_{counts[item]}")
    return new_list

