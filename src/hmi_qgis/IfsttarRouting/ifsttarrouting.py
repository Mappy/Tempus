# -*- coding: utf-8 -*-
"""
/***************************************************************************
 IfsttarRouting
                                 A QGIS plugin
 Get routing informations with various algorithms
                              -------------------
        begin                : 2012-04-12
        copyright            : (C) 2012 by Oslandia
        email                : hugo.mercier@oslandia.com
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
"""
import re
from wps_client import *
from tempus_request import *
import config
import binascii
import pickle
import os
import struct
import math
from datetime import datetime

# Import the PyQt and QGIS libraries
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from qgis.core import *
from qgis.gui import *
#import PyQt4.QtCore
#import PyQt4.QtGui

#from xml.etree import ElementTree as ET
from lxml import etree as ET

# Initialize Qt resources from file resources.py
import resources_rc
# Import the code for the dialog
from ifsttarroutingdock import IfsttarRoutingDock
# Import the splash screen
from ui_splash_screen import Ui_SplashScreen

from history_file import ZipHistoryFile
from result_selection import ResultSelection
from altitude_profile import AltitudeProfile

import pickle

HISTORY_FILE = os.path.expanduser('~/.ifsttarrouting.db')

ROADMAP_LAYER_NAME = "Roadmap_"

# There has been an API change regarding vector layer on 1.9 branch
NEW_API = 'commitChanges' in dir(QgsVectorLayer)

class WKB:

    def __init__( self, wkb ):
        self.wkb = wkb
        # prefix for unpack : little or big endian
        self.prefix = '<' if wkb[1] == '1' else '>'
        self.type = struct.unpack(self.prefix+'h', self.wkb[1*2:3*2].decode('hex'))[0]

    # convert a LineStringZ to a 2D LineString
    def force2d( self ):
        if self.type != 1002: # LineStringZ
            return
        nwkb = self.wkb[0:2] + '0200' + self.wkb[3*2:9*2]
        npts = struct.unpack(self.prefix+'h', self.wkb[5*2:7*2].decode('hex'))[0]
        for i in range(0,npts):
            p = 9+i*3*8
            xs = self.wkb[p*2:(p+8)*2]
            ys = self.wkb[(p+8)*2:(p+16)*2]
            nwkb += xs + ys
        return nwkb

    # get an array of (x,y,z) from a LineStringZ
    def dumpPoints( self ):
        if self.type != 1002:
            return []
        ret = []
        npts = struct.unpack(self.prefix+'h', self.wkb[5*2:7*2].decode('hex'))[0]
        for i in range(0,npts):
            p = 9+i*3*8
            xs = self.wkb[p*2:(p+8)*2]
            ys = self.wkb[(p+8)*2:(p+16)*2]
            zs = self.wkb[(p+16)*2:(p+24)*2]
            x = struct.unpack(self.prefix+'d', xs.decode('hex'))[0]
            y = struct.unpack(self.prefix+'d', ys.decode('hex'))[0]
            z = struct.unpack(self.prefix+'d', zs.decode('hex'))[0]
            ret.append( (x,y,z) )
        return ret

def format_cost( cost ):
    cost_value = float(cost.attrib['value'])
    id = int(cost.attrib['type'])
    return "%s: %.1f %s" % (CostName[id], cost_value, CostUnit[id])

#
# clears a FormLayout
def clearLayout( lay ):
    # clean the widget list
    rc = lay.rowCount()
    if rc > 0:
        for row in range(0, rc):
            l1 = lay.itemAt( rc-row-1, QFormLayout.LabelRole )
            l2 = lay.itemAt( rc-row-1, QFormLayout.FieldRole )
            if l1 is None or l2 is None:
                break
            lay.removeItem( l1 )
            lay.removeItem( l2 )
            w1 = l1.widget()
            w2 = l2.widget()
            lay.removeWidget( w1 )
            lay.removeWidget( w2 )
            w1.close()
            w2.close()

#
# clears a BoxLayout
def clearBoxLayout( lay ):
    rc = lay.count()
    if rc == 0:
        return
    for row in range(0, rc):
        # remove in reverse
        item = lay.itemAt(rc - row - 1)
        lay.removeItem(item)
        w = item.widget()
        lay.removeWidget(w)
        w.close()

class SplashScreen(QDialog):
    def __init__(self):
        QDialog.__init__(self)
        self.ui = Ui_SplashScreen()
        self.ui.setupUi(self)

class IfsttarRouting:

    def __init__(self, iface):
        # Save reference to the QGIS interface
        self.iface = iface
        # Create the dialog and keep reference
        self.canvas = self.iface.mapCanvas()
        self.dlg = IfsttarRoutingDock( self.canvas )
        # show the splash screen
        self.splash = SplashScreen()

        # initialize plugin directory
        self.plugin_dir = QFileInfo(QgsApplication.qgisUserDbFilePath()).path() + "/python/plugins/ifsttarrouting"
        # initialize locale
        localePath = ""
        locale = QSettings().value("locale/userLocale")[0:2]
       
        if QFileInfo(self.plugin_dir).exists():
            localePath = self.plugin_dir + "/i18n/ifsttarrouting_" + locale + ".qm"

        if QFileInfo(localePath).exists():
            self.translator = QTranslator()
            self.translator.load(localePath)

            if qVersion() > '4.3.3':
                QCoreApplication.installTranslator(self.translator)

        self.wps = None
        self.historyFile = ZipHistoryFile( HISTORY_FILE )

        # list of transport types
        self.transport_types = {}
        #list of public networks
        self.networks = {}

        # list of things to save onto history
        self.save = {}

        # where to store plugin options
        self.plugin_options = {}

        # option desc
        self.options = None

        self.currentRoadmap = None

        # prevent reentrance when roadmap selection is in progress
        self.selectionInProgress = False

        self.setStateText("DISCONNECTED")

    def initGui(self):
        # Create action that will start plugin configuration
        self.action = QAction(QIcon(":/plugins/ifsttarrouting/icon.png"), \
            u"Compute route", self.iface.mainWindow())
        # connect the action to the run method
        QObject.connect(self.action, SIGNAL("triggered()"), self.run)

        QObject.connect(self.dlg.ui.connectBtn, SIGNAL("clicked()"), self.onConnect)
        QObject.connect(self.dlg.ui.computeBtn, SIGNAL("clicked()"), self.onCompute)
        QObject.connect(self.dlg.ui.verticalTabWidget, SIGNAL("currentChanged( int )"), self.onTabChanged)

        QObject.connect( self.dlg.ui.pluginCombo, SIGNAL("currentIndexChanged(int)"), self.update_plugin_options )

        # double click on a history's item
        QObject.connect( self.dlg.ui.historyList, SIGNAL("itemDoubleClicked(QListWidgetItem *)"), self.onHistoryItemSelect )
        
        # click on the 'reset' button in history tab
        QObject.connect( self.dlg.ui.reloadHistoryBtn, SIGNAL("clicked()"), self.loadHistory )
        QObject.connect( self.dlg.ui.deleteHistoryBtn, SIGNAL("clicked()"), self.onDeleteHistoryItem )
        QObject.connect( self.dlg.ui.importHistoryBtn, SIGNAL("clicked()"), self.onImportHistory )

        # click on a result radio button
        QObject.connect(ResultSelection.buttonGroup, SIGNAL("buttonClicked(int)"), self.onResultSelected )

        QObject.connect( self.dlg.ui.roadmapTable, SIGNAL("itemSelectionChanged()"), self.onRoadmapSelectionChanged )

        self.originPoint = QgsPoint()
        self.destinationPoint = QgsPoint()

        # Add toolbar button and menu item
        self.iface.addToolBarIcon(self.action)
        self.iface.addPluginToMenu(u"&Compute route", self.action)
        self.iface.addDockWidget( Qt.LeftDockWidgetArea, self.dlg )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 1, False )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 2, False )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 3, False )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 4, False )

        # init the history
        self.loadHistory()

    def setStateText( self, text ):
        self.dlg.setWindowTitle( "Routing - " + text + "" )

    # when the 'connect' button gets clicked
    def onConnect( self ):
        # get the wps url
        g = re.search( 'http://([^/]+)(.*)', str(self.dlg.ui.wpsUrlText.text()) )
        host = g.group(1)
        path = g.group(2)
        connection = HttpCgiConnection( host, path )
        self.wps = WPSClient(connection)

        self.getPluginList()
        
        self.dlg.ui.pluginCombo.setEnabled( True )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 1, True )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 2, True )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 3, True )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 4, True )
        self.dlg.ui.computeBtn.setEnabled( True )
        self.setStateText("connected")

        # restore the history to HISTORY_FILE, if needed (after import)
        if self.historyFile.filename != HISTORY_FILE:
            self.historyFile = ZipHistoryFile( HISTORY_FILE )
            self.loadHistory()

    def getPluginList( self ):
        # get plugin list
        try:
            outputs = self.wps.execute( 'plugin_list', {} )
        except RuntimeError as e:
            QMessageBox.warning( self.dlg, "Error", e.args[1] )
            return

        self.displayPlugins( outputs['plugins'], 0 )
        self.save['plugins'] = to_pson(outputs['plugins'])

    def update_plugin_options( self, plugin_idx ):
        if self.dlg.ui.verticalTabWidget.currentIndex() != 1:
            return

        plugin_name = str(self.dlg.ui.pluginCombo.currentText())
        options_desc = self.options[plugin_name]
        self.displayPluginOptions( plugin_name, options_desc, self.plugin_options[plugin_name] )

    def onTabChanged( self, tab ):
        # Plugin tab
        if tab == 1:
            self.update_plugin_options( 0 )

        # 'Query' tab
        elif tab == 2:
            if self.wps is None:
                return
            try:
                outputs = self.wps.execute( 'constant_list', {} )
            except RuntimeError as e:
                QMessageBox.warning( self.dlg, "Error", e.args[1] )
                return

            self.save['road_types'] = to_pson(outputs['road_types'])
            self.save['transport_types'] = to_pson(outputs['transport_types'])
            self.save['transport_networks'] = to_pson(outputs['transport_networks'])
            self.displayTransportAndNetworks( outputs['transport_types'], outputs['transport_networks'] )

    def loadHistory( self ):
        #
        # History tab
        #
        self.dlg.ui.historyList.clear()
        r = self.historyFile.getRecordsSummary()
        for record in r:
            id = record[0]
            date = record[1]
            dt = datetime.strptime( date, "%Y-%m-%dT%H:%M:%S.%f" )
            item = QListWidgetItem()
            item.setText(dt.strftime( "%Y-%m-%d at %H:%M:%S" ))
            item.setData( Qt.UserRole, int(id) )
            self.dlg.ui.historyList.addItem( item )

    def onDeleteHistoryItem( self ):
        msgBox = QMessageBox()
        msgBox.setIcon( QMessageBox.Warning )
        msgBox.setText( "Warning" )
        msgBox.setInformativeText( "Are you sure you want to remove these items from the history ?" )
        msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        ret = msgBox.exec_()
        if ret == QMessageBox.No:
            return

        sel = self.dlg.ui.historyList.selectedIndexes()
        for item in sel:
            id = item.data( Qt.UserRole )
            self.historyFile.removeRecord( id )
        # reload
        self.loadHistory()

    def onOptionChanged( self, plugin_name, option_name, option_type, val ):
        # bool
        if option_type == 0:
            val = 'true' if val else 0

        self.plugin_options[plugin_name][option_name] = val

    #
    # Take a XML tree from the WPS 'result' operation
    # add an 'Itinerary' layer on the map
    #
    def displayRoadmapLayer(self, steps, lid):
        #
        # create layer

        lname = "%s%d" % (ROADMAP_LAYER_NAME, lid)
        # create a new vector layer
        vl = QgsVectorLayer("LineString?crs=epsg:2154", lname, "memory")

        # road steps are red
        # public transport steps are blue
        # others are yellow
        root_rule = QgsRuleBasedRendererV2.Rule( QgsLineSymbolV2.createSimple({'width': '1.5', 'color' : '255,255,0'} ))
        rule1 = QgsRuleBasedRendererV2.Rule( QgsLineSymbolV2.createSimple({'width': '1.5', 'color' : '255,0,0'}),
                                             0,
                                             0,
                                             "transport_type=1" )
        rule2 = QgsRuleBasedRendererV2.Rule( QgsLineSymbolV2.createSimple({'width': '1.5', 'color' : '0,0,255'}),
                                             0,
                                             0,
                                             "transport_type=2" )
        root_rule.appendChild( rule1 )
        root_rule.appendChild( rule2 )
        renderer = QgsRuleBasedRendererV2( root_rule );
        vl.setRendererV2( renderer )

        pr = vl.dataProvider()

        if NEW_API:
            vl.startEditing()
            vl.addAttribute( QgsField( "transport_type", QVariant.Int ) )
        else:
            pr.addAttributes( [ QgsField( "transport_type", QVariant.Int ) ] )

        # browse steps
        for step in steps:
            if step.tag == 'cost':
                continue
            # find wkb geometry
            wkb = WKB(step.attrib['wkb'])
            wkb = wkb.force2d()

            fet = QgsFeature()
            geo = QgsGeometry()
            geo.fromWkb( binascii.unhexlify(wkb) )
            if step.tag == 'road_step':
                transport_type = 1
            elif step.tag == 'public_transport_step':
                transport_type = 2
            else:
                transport_type = 3

            if NEW_API:
                fet.setAttributes( [ transport_type ] )
            else:
                fet.setAttributeMap( { 0: transport_type } )
            fet.setGeometry( geo )
            pr.addFeatures( [fet] )

        if NEW_API:
            vl.commitChanges()
        else:
            # We MUST call this manually (see http://hub.qgis.org/issues/4687)
            vl.updateFieldMap()
        # update layer's extent when new features have been added
        # because change of extent in provider is not propagated to the layer
        vl.updateExtents()

        QgsMapLayerRegistry.instance().addMapLayers( [vl] )
        self.selectRoadmapLayer( lid )

        # connect selection change signal
        QObject.connect( vl, SIGNAL("selectionChanged()"), lambda layer=vl: self.onLayerSelectionChanged(layer) )

    #
    # Select the roadmap layer
    #
    def selectRoadmapLayer( self, id ):
        legend = self.iface.legendInterface()
        lname = "%s%d" % (ROADMAP_LAYER_NAME, id)
        maps = QgsMapLayerRegistry.instance().mapLayers()
        for k,v in maps.items():
            if v.name()[0:len(ROADMAP_LAYER_NAME)] == ROADMAP_LAYER_NAME:
                if v.name() == lname:
                    legend.setLayerVisible( v, True )
                else:
                    legend.setLayerVisible( v, False )
    #
    # Take a XML tree from the WPS 'result' operation
    # and fill the "roadmap" tab
    #
    def displayRoadmapTab( self, steps ):
        last_movement = 0
        roadmap = steps
        row = 0
        self.dlg.ui.roadmapTable.clear()
        self.dlg.ui.roadmapTable.setRowCount(0)
        self.dlg.ui.roadmapTable.setHorizontalHeaderLabels( ["", "Direction", "Costs"] )

        # array of altitudes (distance, elevation)

        # get or create the graphics scene
        w = self.dlg.ui.resultSelectionLayout
        lastWidget = w.itemAt( w.count() - 1 ).widget()
        if isinstance(lastWidget, AltitudeProfile):
            profile = lastWidget
        else:
            profile = AltitudeProfile( self.dlg )
            self.dlg.ui.resultSelectionLayout.addWidget( profile )

        # mousevent even when no button is clicked

        profile.scene().clear()

        for step in roadmap:
            text = ''
            icon_text = ''
            cost_text = ''

            if step.attrib.has_key('wkb'):
                wkb = WKB(step.attrib['wkb'])
                pts = wkb.dumpPoints()
                prev = pts[0]
                for p in pts[1:]:
                    dist = math.sqrt((p[0]-prev[0])**2 + (p[1]-prev[1])**2)
                    profile.addElevation( dist, prev[2], p[2] )
                    prev = p

            if step.tag == 'road_step':
                road_name = step.attrib['road']
                movement = int(step.attrib['end_movement'])
                costs = step
                text += "<p>"
                action_txt = 'Walk on '
                if last_movement == 1:
                    icon_text += "<img src=\"%s/turn_left.png\" width=\"24\" height=\"24\"/>" % config.DATA_DIR
                    action_txt = "Turn left on "
                elif last_movement == 2:
                    icon_text += "<img src=\"%s/turn_right.png\" width=\"24\" height=\"24\"/>" % config.DATA_DIR
                    action_txt = "Turn right on "
                elif last_movement >= 4 and last_movement < 999:
                    icon_text += "<img src=\"%s/roundabout.png\" width=\"24\" height=\"24\"/>" % config.DATA_DIR
                text += action_txt + road_name + "<br/>\n"
                text += "</p>"
                last_movement = movement

            elif step.tag == 'public_transport_step':
                network = step.attrib['network']
                departure = step.attrib['departure_stop']
                arrival = step.attrib['arrival_stop']
                trip = step.attrib['trip']
                costs = step
                # set network text as icon
                icon_text = network
                text = "Take the trip %s from '%s' to '%s'" % (trip, departure, arrival)

            elif step.tag == 'road_transport_step':
                stype = int(step.attrib['type'])
                road = step.attrib['road']
                icon_text = step.attrib['network']
                stop = step.attrib['stop']
                costs = step
                if stype == 2:
                    text = "Go to the '%s' station from %s" % (stop, road)
                else:
                    text = "Leave the '%s' station to %s" % (stop, road)

            for cost in costs:
                cost_text += format_cost( cost ) + "<br/>\n"
                
            self.dlg.ui.roadmapTable.insertRow( row )
            descLbl = QLabel()
            descLbl.setText( text )
            descLbl.setMargin( 5 )
            self.dlg.ui.roadmapTable.setCellWidget( row, 1, descLbl )

            if icon_text != '':
                lbl = QLabel()
                lbl.setText( icon_text )
                lbl.setMargin(5)
                self.dlg.ui.roadmapTable.setCellWidget( row, 0, lbl )

            costText = QLabel()
            costText.setText( cost_text )
            costText.setMargin( 5 )
            self.dlg.ui.roadmapTable.setCellWidget( row, 2, costText )
            self.dlg.ui.roadmapTable.resizeRowToContents( row )
            row += 1

#        profile.autoFit()

        # Adjust column widths
        w = self.dlg.ui.roadmapTable.sizeHintForColumn(0)
        self.dlg.ui.roadmapTable.horizontalHeader().resizeSection( 0, w )
        w = self.dlg.ui.roadmapTable.sizeHintForColumn(1)
        self.dlg.ui.roadmapTable.horizontalHeader().resizeSection( 1, w )
        w = self.dlg.ui.roadmapTable.sizeHintForColumn(2)
        self.dlg.ui.roadmapTable.horizontalHeader().resizeSection( 2, w )
        self.dlg.ui.roadmapTable.horizontalHeader().setStretchLastSection( True )

    #
    # Take a XML tree from the WPS 'metrics' operation
    # and fill the 'Metrics' tab
    #
    def displayMetrics( self, metrics ):
        row = 0
        clearLayout( self.dlg.ui.resultLayout )

        for metric in metrics:
            lay = self.dlg.ui.resultLayout
            lbl = QLabel( self.dlg )
            lbl.setText( metric.attrib['name'] + '' )
            lay.setWidget( row, QFormLayout.LabelRole, lbl )
            widget = QLineEdit( self.dlg )
            widget.setText( metric.attrib['value'] + '' )
            widget.setEnabled( False )
            lay.setWidget( row, QFormLayout.FieldRole, widget )
            row += 1

    #
    # Take XML trees of 'plugins' and an index of selection
    # and fill the 'plugin' tab
    #
    def displayPlugins( self, plugins, selection ):
        self.dlg.ui.pluginCombo.clear()
        self.options = {}
        for plugin in plugins:
            plugin_name = plugin.attrib['name']

            # save option description
            self.options[plugin_name] = plugin

            # initialize options
            opt_values = {}
            for option in plugin:
                k = option.attrib['name']
                opt_values[k] = option.attrib['default_value']
            self.plugin_options[plugin_name] = opt_values

            self.dlg.ui.pluginCombo.insertItem(0, plugin_name )
        self.dlg.ui.pluginCombo.setCurrentIndex( selection )

    #
    # Take XML tree of 'options' and a dict 'option_values'
    # and fill the options part of the 'plugin' tab
    #
    def displayPluginOptions( self, plugin_name, options_desc, options_value ):
        lay = self.dlg.ui.optionsLayout
        clearLayout( lay )

        row = 0
        for option in options_desc:
            lbl = QLabel( self.dlg )
            name = option.attrib['name'] + ''
            lbl.setText( option.attrib['description'] )
            lay.setWidget( row, QFormLayout.LabelRole, lbl )
            
            t = int(option.attrib['type'])
            
            val = options_value[name]
            # bool type
            if t == 0:
                widget = QCheckBox( self.dlg )
                if val == 'true':
                    widget.setCheckState( Qt.Checked )
                else:
                    widget.setCheckState( Qt.Unchecked )
                QObject.connect(widget, SIGNAL("toggled(bool)"), lambda checked, name=name, t=t, pname=plugin_name: self.onOptionChanged( pname, name, t, checked ) )
            else:
                widget = QLineEdit( self.dlg )
                if t == 1:
                    valid = QIntValidator( widget )
                    widget.setValidator( valid )
                if t == 2:
                    valid = QDoubleValidator( widget )
                    widget.setValidator( valid )
                widget.setText( val )
                QObject.connect(widget, SIGNAL("textChanged(const QString&)"), lambda text, name=name, t=t, pname=plugin_name: self.onOptionChanged( pname, name, t, text ) )
            lay.setWidget( row, QFormLayout.FieldRole, widget )
            
            row += 1

    #
    # Take an XML trees from 'constant_list'
    # load them and display them
    def displayTransportAndNetworks( self, xmlTransportTypes, xmlTransportNetworks ):
            #
            # add each transport type to the list
            #

            # retrieve the list of transport types
            self.transport_types = []
            for transport_type in xmlTransportTypes:
                self.transport_types.append(transport_type.attrib)

            # retrieve the network list
            self.networks = []
            for network in xmlTransportNetworks:
                self.networks.append(network.attrib)

            listModel = QStandardItemModel()
            for ttype in self.transport_types:
                need_network = int(ttype['need_network'])
                idt = int(ttype['id'])
                has_network = False
                if need_network:
                    # look for networks that provide this kind of transport
                    for network in self.networks:
                        ptt = int(network['provided_transport_types'])
                        if ptt & idt:
                            has_network = True
                            break

                item = QStandardItem( ttype['name'] )
                if need_network and not has_network:
                    item.setFlags(Qt.ItemIsUserCheckable)
                    item.setData( Qt.Unchecked, Qt.CheckStateRole)
                else:
                    item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
                    item.setData( Qt.Checked, Qt.CheckStateRole)
                listModel.appendRow(item)
            self.dlg.ui.transportList.setModel( listModel )

            networkListModel = QStandardItemModel()
            for network in self.networks:
                title = network['name']

                # look for provided transport types
                tlist = []
                ptt = int( network['provided_transport_types'] )
                for ttype in self.transport_types:
                    if ptt & int(ttype['id']):
                        tlist.append( ttype['name'] )

                item = QStandardItem( network['name'] + ' (' + ', '.join(tlist) + ')')
                item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
                item.setData( Qt.Checked, Qt.CheckStateRole)
                networkListModel.appendRow(item)
            self.dlg.ui.networkList.setModel( networkListModel )

    #
    # Take an XML tree from the WPS 'results'
    # and load/display them
    #
    def displayResults( self, results ):
        # result radio button id
        self.result_ids=[]
        # clear the vbox layout
        clearBoxLayout( self.dlg.ui.resultSelectionLayout )
        k = 1
        for result in results:
            name = "%s%d" % (ROADMAP_LAYER_NAME,k)
            rselect = ResultSelection()
            rselect.setText( name )
            for r in result:
                if r.tag == 'cost':
                    # get the roadmap
                    id = int(r.attrib['type'])
                    rselect.addCost(CostName[id], "%.1f%s" % (float(r.attrib['value']), CostUnit[id]))
            self.dlg.ui.resultSelectionLayout.addWidget( rselect )
            self.result_ids.append( rselect.id() )
            k += 1
        self.results = results

        # delete pre existing roadmap layers
        maps = QgsMapLayerRegistry.instance().mapLayers()
        for k,v in maps.items():
            if v.name()[0:len(ROADMAP_LAYER_NAME)] == ROADMAP_LAYER_NAME:
                QgsMapLayerRegistry.instance().removeMapLayers( [k] )

        # then display each layer
        k = 1
        for result in results:
            self.displayRoadmapLayer( result, k )
            k += 1

    def onResultSelected( self, id ):
        for i in range(0, len(self.result_ids)):
            if id == self.result_ids[i]:
                self.displayRoadmapTab( self.results[i] )
                self.selectRoadmapLayer( i+1 )
                self.currentRoadmap = i
                break

    #
    # When the 'compute' button gets clicked
    #
    def onCompute(self):
        if self.wps is None:
            return

        # retrieve request data from dialogs
        coords = self.dlg.get_coordinates()
        [ox, oy] = coords[0]
        criteria = self.dlg.get_selected_criteria()
        constraints = self.dlg.get_constraints()
        parking = self.dlg.get_parking()
        pvads = self.dlg.get_pvads()
        networks = [ self.networks[x] for x in self.dlg.selected_networks() ]
        transports = [ self.transport_types[x] for x in self.dlg.selected_transports() ]

        # build the request

        allowed_transports = 0
        for x in transports:
            allowed_transports += int(x['id'])

        r = [ 'request',
              { 'allowed_transport_types': allowed_transports },
              ['origin', {'x': str(ox), 'y': str(oy) } ],
              ['departure_constraint', { 'type': constraints[0][0], 'date_time': constraints[0][1] } ]]

        if parking != []:
            r.append(['parking_location', {'x': parking[0], 'y': parking[1]} ])

        for criterion in criteria:
            r.append(['optimizing_criterion', criterion])

        for n in networks:
            r.append( ['allowed_network', int(n['id']) ] )

        n = len(constraints)
        for i in range(1, n):
            pvad = 'false'
            if pvads[i-1] == True:
                pvad = 'true'
            r.append( ['step',
                       { 'private_vehicule_at_destination': pvad },
                       [ 'destination', {'x': str(coords[i][0]), 'y': str(coords[i][1])} ],
                       [ 'constraint', { 'type' : constraints[i][0], 'date_time': constraints[i][1] } ],
                       ] )

        currentPluginIdx = self.dlg.ui.pluginCombo.currentIndex()
        currentPlugin = str(self.dlg.ui.pluginCombo.currentText())
        plugin_arg = { 'plugin' : ['plugin', {'name' : currentPlugin } ] }

        args = plugin_arg
        args['request'] = r

        # plugin options processing
        options = [ [ 'option', { 'name':k, 'value':str(v)}] for k,v in self.plugin_options[currentPlugin].iteritems() ]
        options.insert( 0, 'options' )
        args['options'] = options

        try:
            outputs = self.wps.execute( 'select', args )
        except RuntimeError as e:
            QMessageBox.warning( self.dlg, "Error", e.args[1] )
            return

        self.displayResults( outputs['results'] )
        self.displayMetrics( outputs['metrics'] )

        # enable 'metrics' and 'roadmap' tabs
        self.dlg.ui.verticalTabWidget.setTabEnabled( 3, True )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 4, True )
        # clear the roadmap table
        self.dlg.ui.roadmapTable.clear()
        self.dlg.ui.roadmapTable.setRowCount(0)

        # save request state
        request = [ 'select', args['plugin'], args['request'], args['options'], to_pson(outputs['results']), to_pson(outputs['metrics']) ]

        # save server state (in self.save)
        server_state = [ 'server_state', self.save['plugins'], self.save['road_types'], 
                         self.save['transport_types'], self.save['transport_networks'] ]
        pson = [ 'record', request, server_state ]

        str_record = to_xml(pson)
        self.historyFile.addRecord( str_record )

    def onHistoryItemSelect( self, item ):
        msgBox = QMessageBox()
        msgBox.setIcon( QMessageBox.Warning )
        msgBox.setText( "Warning" )
        msgBox.setInformativeText( "Do you want to load this item from the history ? Current options and query parameters will be overwritten" )
        msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        ret = msgBox.exec_()
        if ret == QMessageBox.No:
            return

        id = item.data( Qt.UserRole )
        # load from db
        (id, date, xmlStr) = self.historyFile.getRecord( id )

        # validate entry
        schema_root = ET.parse( config.TEMPUS_DATA_DIR + "/wps_schemas/record.xsd" )
        schema = ET.XMLSchema(schema_root)
        parser = ET.XMLParser(schema = schema)
        try:
            tree = ET.fromstring( xmlStr, parser)
        except ET.XMLSyntaxError as e:
            QMessageBox.warning( self.dlg, "XML parsing error", e.msg )
            return

#        tree = ET.XML(xmlStr)
        loaded = {}
        for child in tree:
            loaded[ child.tag ] = child

        # update UI
        self.dlg.loadFromXML( loaded['select'][1] )
        self.displayMetrics( loaded['select'][4] )
        self.displayResults( loaded['select'][3] )

        self.displayPlugins( loaded['server_state'][0], 0);
        
        # get current plugin option
        currentPlugin = loaded['select'][0].attrib['name']
        plugin_options = self.plugin_options[currentPlugin]
        options = loaded['select'][2]
        for option in options:
            k = option.attrib['name']
            v = option.attrib['value']
            plugin_options[k] = v

        self.transport_types = loaded['server_state'][2]
        self.networks = loaded['server_state'][3]
        self.displayTransportAndNetworks( self.transport_types, self.networks )

        # enable tabs
        self.dlg.ui.verticalTabWidget.setTabEnabled( 1, True )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 2, True )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 3, True )
        self.dlg.ui.verticalTabWidget.setTabEnabled( 4, True )
        
        # simulate a disconnection
        self.wps = None
        self.dlg.ui.computeBtn.setEnabled(False)
        self.setStateText("DISCONNECTED")

    def onImportHistory( self ):
        fname = QFileDialog.getOpenFileName( None, 'Import history file', '')
        if not fname:
            return

        # switch historyFile
        self.historyFile = ZipHistoryFile( fname )
        self.loadHistory()

    # when the user selects a row of the roadmap
    def onRoadmapSelectionChanged( self ):
        if self.selectionInProgress:
            return

        selected = self.dlg.ui.roadmapTable.selectedIndexes()
        rows = []
        for s in selected:
            r = s.row()
            if r not in rows: rows.append( r )

        roadmap = self.results[ self.currentRoadmap ]

        # find layer
        layer = None
        lname = "%s%d" % (ROADMAP_LAYER_NAME, self.currentRoadmap)
        maps = QgsMapLayerRegistry.instance().mapLayers()
        for k,v in maps.items():
            if v.name()[0:len(ROADMAP_LAYER_NAME)] == ROADMAP_LAYER_NAME:
                layer = v
                break
        # the layer may have been deleted
        if not layer:
            return

        self.selectionInProgress = True
        layer.selectAll()
        layer.invertSelection() # no deselectAll ??
        for row in rows:
            # row numbers start at 0 and ID starts at 1
            layer.select( row + 1)
        self.selectionInProgress = False

    # when selection on the itinerary layer changes
    def onLayerSelectionChanged( self, layer ):
        if self.selectionInProgress:
            return

        n = len(ROADMAP_LAYER_NAME)
        if len(layer.name()) <= n:
            return
        layerid = int(layer.name()[n:])-1
        self.currentRoadmap = layerid

        # in case the layer still exists, but not the underlying result
        if len(self.results) <= layerid:
            return

        # block signals to prevent infinite loop
        self.selectionInProgress = True
        self.displayRoadmapTab( self.results[layerid-1] )

        selected = [ feature.id() for feature in layer.selectedFeatures() ]
        for fid in selected:
            self.dlg.ui.roadmapTable.selectRow( fid-1 )
        self.selectionInProgress = False

    def unload(self):
        # Remove the plugin menu item and icon
        self.iface.removePluginMenu(u"&Compute route",self.action)
        self.iface.removeToolBarIcon(self.action)

    # run method that performs all the real work
    def run(self):
        self.splash.show()
        # show the dialog
        self.dlg.show()
