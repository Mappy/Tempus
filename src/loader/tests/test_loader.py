#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
/**
 *   Copyright (C) 2012-2014 IFSTTAR (http://www.ifsttar.fr)
 *   Copyright (C) 2012-2014 Oslandia <infos@oslandia.com>
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Library General Public
 *   License as published by the Free Software Foundation; either
 *   version 2 of the License, or (at your option) any later version.
 *   
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Library General Public License for more details.
 *   You should have received a copy of the GNU Library General Public
 *   License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
"""

import sys
import os
import unittest
import argparse
import subprocess

script_path = os.path.abspath(os.path.dirname(sys.argv[0]))
loader_path = os.path.abspath(script_path + '/..')
data_path = os.path.abspath( script_path + '/../../../test_data' )
sys.path.insert(0, loader_path)

import tempus
from tempus.config import *

loader = loader_path + "/load_tempus"
dbstring = "dbname=tempus_test_loader"

def get_sql_output( dbstring, sql ):
    cmd = [PSQL, dbstring, '-t', '-c', sql]
    p = subprocess.Popen(cmd, stdout = subprocess.PIPE )
    return p.stdout.read()

class TestTempusLoader(unittest.TestCase):

    def setUp(self):
        pass

    def _test_osm_loading( self ):
        r = subprocess.call( [loader, '-t', 'osm', '-s', data_path, '-d', dbstring, '-R'] )
        self.assertEqual(r, 0)

        # FIXME : not stable !
        n_road_nodes = int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.road_node"))
        self.assertEqual(3113, n_road_nodes)

        n_road_sections = int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.road_section"))
        self.assertEqual(30838, n_road_sections)

    def test_gtfs_loading( self ):
        # GTFS loading without road
        r = subprocess.call( [loader, '-t', 'gtfs', '-s', data_path + '/gtfs.zip', '-d', dbstring, '-R'] )
        self.assertEqual(r, 0)

        self.assertEqual(int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.road_node")), 6830)
        self.assertEqual(int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.road_section")), 3415)
        self.assertEqual(int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.pt_stop")), 3410)
        self.assertEqual(int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.pt_section")), 2659)
        self.assertEqual(int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.pt_trip")), 41128)
        self.assertEqual(int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.pt_stop_time")), 1086613)
        self.assertEqual(int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.pt_route")), 65)

    def test_gtfs_transfers_loading( self ):
        # GTFS loading without road
        r = subprocess.call( [loader, '-t', 'gtfs', '-s', data_path + '/gtfs_with_transfers.zip', '-d', dbstring, '-R'] )
        self.assertEqual(r, 0)

        # number of transfers
        self.assertEqual(int(get_sql_output(dbstring, "SELECT count(*) FROM tempus.road_section WHERE road_type=5")), 5)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Tempus data loader')
    parser.add_argument('-d', '--dbstring', required=False,
            help='The database connection string')
    args = parser.parse_args()

    if args.dbstring is not None:
        dbstring = args.dbstring

    unittest.main()


