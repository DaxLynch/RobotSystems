#!/usr/bin/env python3
'''
**********************************************************************
* Filename    : filedb.py
* Description : A simple file based database.
*               Modified for simulation - skips file operations when not on robot.
* Author      : Cavon
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Cavon    2016-09-13    New release
**********************************************************************
'''
import os
from time import sleep


class fileDB(object):
    """A file based database.

    A file based database, read and write arguements in the specific file.
    Modified for simulation mode - returns defaults without file operations.
    """
    def __init__(self, db:str, mode:str=None, owner:str=None):  
        '''
        Init the db_file is a file to save the datas.
        
        :param db: the file to save the datas.
        :type db: str
        :param mode: the mode of the file.
        :type mode: str
        :param owner: the owner of the file.
        :type owner: str
        '''

        self.db = db
        # Skip file operations in simulation mode
        # self.file_check_create() is not called

    def file_check_create(self, file_path:str, mode:str=None, owner:str=None):
        """
        Check if file is existed, otherwise create one.
        Skipped in simulation mode.
        """
        pass

    def get(self, name, default_value=None):
        """
        Get value with data's name.
        In simulation mode, always returns default_value.
        
        :param name: the name of the arguement
        :type name: str
        :param default_value: the default value of the arguement
        :type default_value: str
        :return: the value of the arguement
        :rtype: str
        """
        return default_value

    def set(self, name, value):
        """
        Set value by with name.
        In simulation mode, does nothing.
        
        :param name: the name of the arguement
        :type name: str
        :param value: the value of the arguement
        :type value: str
        """
        pass


if __name__ == '__main__':
    db = fileDB('/opt/robot-hat/test2.config')

    db.set('a', '1')
    db.set('b', '2')

    print(db.get('a'))
    print(db.get('c'))
