
from shapely.geometry import Polygon, LinearRing, LineString, MultiPolygon, Point
import copy
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
import numpy as np
import pickle
#import os


# Koordinaten aus dem Slicer für jede Schicht
# Umwandeln des Numpy Arrays in Shapely-Polygone

class GetCoords:

    def __init__(self, slices_of_all_layers):
        self.slices_of_all_layers = slices_of_all_layers

    # Ermitteln der Polygone der äußeren Geometrie :
    def get_outer_shells(self):

        all_outer_polygons = []

        for shells_of_layer in self.slices_of_all_layers:

            polygons_in_this_layer = []

            for shell in shells_of_layer:

                for outline in shell:

                    # x- ,y- ,z-Werte durch transponieren des Numpy Array
                    x_values = (np.transpose(outline)[0])
                    y_values = (np.transpose(outline)[1])
                    z_values = (np.transpose(outline)[2])

                    x_y_z_list = []

                    # In für Shapely-kompatibles Format formatieren
                    for x, y, z in zip(x_values, y_values, z_values):
                        x_y_z_tuples = []

                        x_y_z_tuples.append((round(x, 3)))
                        x_y_z_tuples.append((round(y, 3)))
                        x_y_z_tuples.append((round(z, 3)))

                        x_y_z_list.append(x_y_z_tuples)

                    if len(x_y_z_list) >= 3:

                        # Shapely-Polygon erstellen
                        outer_polygon = Polygon(x_y_z_list)

                        # Prüfen, ob das Polygon richtig erstellt wurde
                        if outer_polygon.is_valid:
                            polygons_in_this_layer.append(outer_polygon)

                    # Fehlermeldungen > Falls der Slicer falsche Werte übergibt
                        else:
                            print("!!!!!!!!!!!!!!!!!!!!OUTER GEOMETRY ERROR: Invalid Polygon !!!!!!!!!!!!!!!!!!!!!!!!!!!!!")


                    else:
                        print("!!!!!!!!!!!!!!!!!!!!!!!!OUTER GEOMETRY ERROR: Too less Points!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

                    # Break > Informationen für die Außerngeometrien sind nur in der ersten Liste von slices_of_all_layers
                    break

            all_outer_polygons.append(polygons_in_this_layer)

        # Alle Polygone einer Schicht befinden sich immer in einer Liste
        return all_outer_polygons


    # Ermitteln der Polygone in der Innengeometrie
    def get_inner_shells(self):

        global polygons_in_this_layer
        all_inner_polygons = []

        for shells_of_layer in self.slices_of_all_layers:

            inner_polygons_in_this_layer = []

            for shell in shells_of_layer:

                # Innengeometrien befinden sich ab der zweiten Liste von slices_of_all_layers
                counter = 0
                for outline in shell:

                    if counter > 0:

                        x_values = (np.transpose(outline)[0])
                        y_values = (np.transpose(outline)[1])
                        z_values = (np.transpose(outline)[2])

                        x_y_z_list = []

                        for x, y, z in zip(x_values, y_values, z_values):
                            x_y_z_tuples = []

                            x_y_z_tuples.append((round(x, 3)))
                            x_y_z_tuples.append((round(y, 3)))
                            x_y_z_tuples.append((round(z, 3)))

                            x_y_z_list.append(x_y_z_tuples)

                        if len(x_y_z_list) >= 3:
                            inner_polygon = Polygon(x_y_z_list)

                            if inner_polygon.is_valid:
                                inner_polygons_in_this_layer.append(inner_polygon)

                            else:
                                print("!!!!!!!!!!!!!!INNER GEOMETRY ERROR: Invalid Polygon!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

                        else:
                            print("!!!!!!!!!!!!!!!!!!INNER GEOMETRY ERROR: Too less Points!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

                    counter += 1

            all_inner_polygons.append(inner_polygons_in_this_layer)

        # Alle Polygone einer Schicht befinden sich in einer Liste
        return all_inner_polygons






# Ermittlung der kritischen Überhänge:
class Angle_Analysis:

    def __init__(self, critical_angle_1, critical_angle_2, layer_thickness):
        self.critical_angle_1 = critical_angle_1
        self.critical_angle_2 = critical_angle_2
        self.layer_thickness = layer_thickness

        #self.read_out_read_out_cache_list_outside = []

        self.critical_outer_overhangs = []
        self.critical_inner_overhangs = []

        self.critical_outer_overhangs_2 = []


    # Winkel der äußeren Geometrie messen
    def measure_critical_distance_outside(self, poly_list):

        global z_value_layer_n_1, layer_n_1, read_out_read_out_cache_list


        # kritischer Downskin-Winkel von Grad in rad
        critical_angle_1_rad = (self.critical_angle_1 * np.pi) / 180
        critical_angle_2_rad = (self.critical_angle_2 * np.pi) / 180

        # Aus kritischem Winkel kritische Distanz berechnen
        if np.tan(critical_angle_1_rad) == 0:
            critical_distance_1 = 10000000000000000000 # Verhindert teilen durch 0
        else:
            critical_distance_1 = round(self.layer_thickness / np.tan(critical_angle_1_rad), 5)

        if np.tan(critical_angle_2_rad) == 0:
            critical_distance_2 = 10000000000000000000
        else:
            critical_distance_2 = round(self.layer_thickness / np.tan(critical_angle_2_rad), 5)

        # Jeweils die aktuelle Schicht (polygone_n) und die darüberliegende Schicht (polygone_n_1) auf
        # kritische Überhänge überprüfen

        # Weil das erste Element in der Liste leer ist, bei zweitem Element anfangen
        for polygone_n, polygone_n_1 in zip(poly_list[1:], poly_list[2:]):

            counter = 0
            critical_outer_overhang_cache = []

            # Jede Schicht kann aus mehreren Polygnen bestehen
            # Es wird nacheinander jedes Polygon der oben liegenden Schicht (layer_n_1) mit jedem
            # Polygon der darunterliegenden Schicht verglichen
            # Die kritischen Überhänge werden als Polygon zunächst im cirtical_outer_overhang_cache
            # zwischengespeichert. Wenn sich mehrere Polygone der unteren Schicht mit einem Polygon der
            # oberen Schicht schneiden, enstehen mehrere Überhänge, die jedoch zum Teil die gleiche Fläche beschreiben
            # Diese doppelten Werte werden anschließend entfernt



            while counter < len(polygone_n_1):
                seperate_island = []

                for i in range(0, len(polygone_n)):
                    layer_n = polygone_n[i]
                    layer_n_1 = polygone_n_1[counter]

                    # Freischwebende Überhänge erkennen
                    if layer_n.contains(layer_n_1) or layer_n_1.contains(layer_n) or layer_n.intersects(layer_n_1):
                        seperate_island.append("True")

                    # Z-Höhe der oberen Schicht
                    if layer_n.type == "Polygon":
                        coords_layer_n_1 = layer_n_1.exterior.coords
                        z_value_layer_n_1 = coords_layer_n_1[0][2]

                    # Schließt Polygone aus, die keinen Überhang haben können
                    if layer_n.type == "Polygon" and layer_n_1.type == "Polyon" and layer_n.contains(layer_n_1):
                        do_nothing = None

                    else:

                        # Erstellt den Überhang:
                        dif = layer_n_1.difference(layer_n)

                        # Überprüft, ob es ein Überhang ist:
                        if dif.equals(layer_n_1) is False:

                            # dif kann den Typ "Polygon" oder "MiltiPolygon" haben:
                            if dif.type == "Polygon" and dif.is_empty is False:

                                # Punkte von dif ermitteln und über diese maximalen Abstand zum unteren Polygon ermitteln
                                # Dies entspricht Länge die Überhang übersteht
                                save_list = []
                                save_list_2 = []
                                coords_list = list(zip(*dif.exterior.xy))

                                for point in coords_list:
                                    points = Point(point)

                                    if layer_n.type == "Polygon":
                                        current_distance = round(points.distance(layer_n), 5)
                                        # Hier könnte man möglicherweise mit hausdorff_distance
                                        # den Code etwas vereinfachen


                                        # Überprüfen, ob der Überhang weiter übersteht als der kritische Wert
                                        # Dabei reicht es aus, wenn ein Punkt den kritischen Wert überschreitet

                                        if current_distance >= critical_distance_1:
                                            save_list.append(True)
                                        else:
                                            save_list.append(False)
                                        if current_distance >= critical_distance_2:
                                            save_list_2.append(True)

                                if True in save_list and not save_list_2:
                                    critical_outer_overhang_cache.append(dif)


                            elif dif.type == "MultiPolygon" and dif.is_empty is False:

                                poly_support_liste = []

                                # Bei MultiPolygon muss jedes der enthaltenen Polygone überprüft werden
                                for polygone in dif:
                                    coords_list = list(zip(*polygone.exterior.coords.xy))

                                    for point in coords_list:
                                        points = Point(point)

                                        if layer_n.type == "Polygon":

                                            current_distance = round(points.distance(layer_n), 5)

                                            if current_distance >= critical_distance_1 and current_distance < critical_distance_2:

                                                if not any(p.equals(Polygon(coords_list)) for p in poly_support_liste):
                                                    poly_support_liste.append(Polygon(coords_list))

                                multi_support = MultiPolygon(poly_support_liste)

                                if multi_support.is_empty is False:
                                    critical_outer_overhang_cache.append(dif)


                if not "True" in seperate_island:
                    critical_outer_overhang_cache.append(layer_n_1)

                counter += 1






            # Doppelte Werte löschen:
            read_out_cache_list = []
            for polygons in critical_outer_overhang_cache:
                if polygons.type == "MultiPolygon":
                    for polys in polygons:
                        read_out_cache_list.append(polys)
                elif polygons.type == "Polygon":
                    read_out_cache_list.append(polygons)



            # Über while-Schleifen vergleichen der Überhänge ob diese Überschneidungen (intersection) haben
            # Es wird nur die Überschneidung behalten und die beiden ursprünglichen Überhänge gelöscht
            # Dazu wird jeder Überhang nacheinander überprüft
            # Beispiel für 4 Überhänge: Ü1, Ü2, Ü3, Ü4.
            # Abgeglichen wird: Ü1-Ü2, Ü1-Ü3, Ü1-Ü4, Ü2-Ü3, Ü2-Ü4, Ü3-Ü4
            # Bei einer Überschneidung wird die while Schleife wieder von vorne durchlaufen und
            # dabei die neue Anzahl an Überhängen berücksichtig

            counter_1 = 0
            counter_2 = 1
            counter_3 = len(read_out_cache_list)

            while counter_1 < counter_3-1:

                while counter_2 < counter_3:

                    intersection_polygon = read_out_cache_list[counter_1].intersection(read_out_cache_list[counter_2])

                    if not intersection_polygon.is_empty:
                        read_out_cache_list_copy = read_out_cache_list[counter_2]
                        if read_out_cache_list[counter_1] in read_out_cache_list:
                            read_out_cache_list.remove(read_out_cache_list[counter_1])

                        if read_out_cache_list_copy in read_out_cache_list:
                            read_out_cache_list.remove(read_out_cache_list_copy)

                        #Alte Methode:
                        # read_out_cache_list_copy = copy.copy(read_out_cache_list)
                        # if read_out_cache_list_copy[counter_1] in read_out_cache_list:
                        #     read_out_cache_list.remove(read_out_cache_list_copy[counter_1])
                        #
                        # if read_out_cache_list_copy[counter_2] in read_out_cache_list:
                        #     read_out_cache_list.remove(read_out_cache_list_copy[counter_2])

                        read_out_cache_list.append(intersection_polygon)


                        counter_1 = 0
                        counter_2 = 1
                        counter_3 = len(read_out_cache_list)

                    else:
                        counter_1 += 1
                        counter_2 += 1
                        counter_3 = len(read_out_cache_list)






            # In Numpy Array abspeichern

            read_out_read_out_cache_list = []
            for polygones in read_out_cache_list:
                if polygones.type is "MultiPolygon":
                    for polys in polygones:
                        read_out_read_out_cache_list.append(polys)
                elif polygones.type == "Polygon":
                    read_out_read_out_cache_list.append(polygones)

            #self.read_out_read_out_cache_list_outside.append(read_out_read_out_cache_list)

            for polygones in read_out_read_out_cache_list:
                poly_coords = list(zip(*polygones.exterior.coords))
                lenght = len(poly_coords[2])

                poly_coords[2] = [z_value_layer_n_1] * lenght


                x_y_z_list = []

                for x, y, z in zip(poly_coords[0], poly_coords[1], poly_coords[2]):
                    x_y_z_tuples = []

                    x_y_z_tuples.append(x)
                    x_y_z_tuples.append(y)
                    x_y_z_tuples.append(z)
                    x_y_z_list.append(x_y_z_tuples)

                coords_array = np.array(x_y_z_list)
                coords_array.reshape(3, lenght)
                self.critical_outer_overhangs.append(coords_array)




    # Winkel 2 der äußeren Geometrie messen

    def measure_critical_distance_outside_2(self, poly_list):

        global z_value_layer_n_1, layer_n_1, read_out_read_out_cache_list

        # kritischer Downskin-Winkel von Grad in rad
        critical_angle_1_rad = (self.critical_angle_1 * np.pi) / 180
        critical_angle_2_rad = (self.critical_angle_2 * np.pi) / 180

        # Aus kritischem Winkel kritische Distanz berechnen
        if np.tan(critical_angle_2_rad) == 0:
            critical_distance_2 = 10000000000000000000
        else:
            critical_distance_2 = round(self.layer_thickness / np.tan(critical_angle_2_rad), 5)

        # Jeweils die aktuelle Schicht (polygone_n) und die darüberliegende Schicht (polygone_n_1) auf
        # kritische Überhänge überprüfen
        loop_counter = 0
        for polygone_n, polygone_n_1 in zip(poly_list, poly_list[1:]):

            counter = 0
            critical_outer_overhang_cache = []

            # Jede Schicht kann aus mehreren Polygnen bestehen
            # Es wird nacheinander jedes Polygon der oben liegenden Schicht (layer_n_1) mit jedem
            # Polygon der darunterliegenden Schicht verglichen
            # Die kritischen Überhänge werden als Polygon zunächst im cirtical_outer_overhang_cache
            # zwischengespeichert. Wenn sich mehrere Polygone der unteren Schicht mit einem Polygon der
            # oberen Schicht schneiden, enstehen mehrere Überhänge, die jedoch zum Teil die gleiche Fläche beschreiben
            # Diese doppelten Werte werden anschließend entfernt

            # Erste Schicht immer kritisch
            if loop_counter == 0:
                for outlines in polygone_n:
                    critical_outer_overhang_cache.append(outlines)

            while counter < len(polygone_n_1):
                seperate_island = []

                for i in range(0, len(polygone_n)):
                    layer_n = polygone_n[i]
                    layer_n_1 = polygone_n_1[counter]

                    # Freischwebende Überhänge erkennen
                    if layer_n.contains(layer_n_1) or layer_n_1.contains(layer_n) or layer_n.intersects(layer_n_1):
                        seperate_island.append("True")

                    # Z-Höhe der oberen Schicht
                    if layer_n.type == "Polygon":
                        coords_layer_n_1 = layer_n_1.exterior.coords
                        z_value_layer_n_1 = coords_layer_n_1[0][2]

                    # Schließt Polygone aus, die keinen Überhang haben können
                    if layer_n.type == "Polygon" and layer_n_1.type == "Polyon" and layer_n.contains(layer_n_1):
                        do_nothing = None

                    else:

                        # Erstellt den Überhang:
                        dif = layer_n_1.difference(layer_n)

                        # Überprüft, ob es ein Überhang ist:
                        if dif.equals(layer_n_1) is False:

                            # dif kann den Typ "Polygon" oder "MiltiPolygon" haben:
                            if dif.type == "Polygon" and dif.is_empty is False:

                                # Punkte von dif ermitteln und über diese maximalen Abstand zum unteren Polygon ermitteln
                                # Dies entspricht Länge die Überhang übersteht
                                save_list = []

                                coords_list = list(zip(*dif.exterior.xy))

                                for point in coords_list:
                                    points = Point(point)

                                    if layer_n.type == "Polygon":
                                        current_distance = round(points.distance(layer_n), 5)
                                        # Hier könnte man möglicherweise mit hausdorff_distance
                                        # den Code etwas vereinfachen

                                        # Überprüfen, ob der Überhang weiter übersteht als der kritische Wert
                                        # Dabei reicht es aus, wenn ein Punkt den kritischen Wert überschreitet

                                        if current_distance >= critical_distance_2:
                                            save_list.append(True)
                                        else:
                                            save_list.append(False)


                                if True in save_list:
                                    critical_outer_overhang_cache.append(dif)


                            elif dif.type == "MultiPolygon" and dif.is_empty is False:

                                poly_support_liste = []

                                # Bei MultiPolygon muss jedes der enthaltenen Polygone überprüft werden
                                for polygone in dif:
                                    coords_list = list(zip(*polygone.exterior.coords.xy))

                                    for point in coords_list:
                                        points = Point(point)

                                        if layer_n.type == "Polygon":

                                            current_distance = round(points.distance(layer_n), 5)

                                            if current_distance >= critical_distance_2:

                                                if not any(p.equals(Polygon(coords_list)) for p in poly_support_liste):
                                                    poly_support_liste.append(Polygon(coords_list))

                                multi_support = MultiPolygon(poly_support_liste)

                                if multi_support.is_empty is False:
                                    critical_outer_overhang_cache.append(dif)

                if not "True" in seperate_island:
                    critical_outer_overhang_cache.append(layer_n_1)

                counter += 1
            loop_counter += 1
            # Doppelte Werte löschen:
            read_out_cache_list = []
            for polygons in critical_outer_overhang_cache:
                if polygons.type == "MultiPolygon":
                    for polys in polygons:
                        read_out_cache_list.append(polys)
                elif polygons.type == "Polygon":
                    read_out_cache_list.append(polygons)

            # Über while-Schleifen vergleichen der Überhänge ob diese Überschneidungen (intersection) haben
            # Es wird nur die Überschneidung behalten und die beiden ursprünglichen Überhänge gelöscht
            # Dazu wird jeder Überhang nacheinander überprüft
            # Beispiel für 4 Überhänge: Ü1, Ü2, Ü3, Ü4.
            # Abgeglichen wird: Ü1-Ü2, Ü1-Ü3, Ü1-Ü4, Ü2-Ü3, Ü2-Ü4, Ü3-Ü4
            # Bei einer Überschneidung wird die while Schleife wieder von vorne durchlaufen und
            # dabei die neue Anzahl an Überhängen berücksichtig

            counter_1 = 0
            counter_2 = 1
            counter_3 = len(read_out_cache_list)

            while counter_1 < counter_3 - 1:

                while counter_2 < counter_3:

                    intersection_polygon = read_out_cache_list[counter_1].intersection(read_out_cache_list[counter_2])

                    if not intersection_polygon.is_empty:
                        read_out_cache_list_copy = read_out_cache_list[counter_2]
                        if read_out_cache_list[counter_1] in read_out_cache_list:
                            read_out_cache_list.remove(read_out_cache_list[counter_1])

                        if read_out_cache_list_copy in read_out_cache_list:
                            read_out_cache_list.remove(read_out_cache_list_copy)

                        # Alte Methode:
                        # read_out_cache_list_copy = copy.copy(read_out_cache_list)
                        # if read_out_cache_list_copy[counter_1] in read_out_cache_list:
                        #     read_out_cache_list.remove(read_out_cache_list_copy[counter_1])
                        #
                        # if read_out_cache_list_copy[counter_2] in read_out_cache_list:
                        #     read_out_cache_list.remove(read_out_cache_list_copy[counter_2])

                        read_out_cache_list.append(intersection_polygon)

                        counter_1 = 0
                        counter_2 = 1
                        counter_3 = len(read_out_cache_list)

                    else:
                        counter_1 += 1
                        counter_2 += 1
                        counter_3 = len(read_out_cache_list)

            # In Numpy Array abspeichern

            read_out_read_out_cache_list = []
            for polygones in read_out_cache_list:
                if polygones.type is "MultiPolygon":
                    for polys in polygones:
                        read_out_read_out_cache_list.append(polys)
                elif polygones.type == "Polygon":
                    read_out_read_out_cache_list.append(polygones)

            # self.read_out_read_out_cache_list_outside.append(read_out_read_out_cache_list)

            for polygones in read_out_read_out_cache_list:
                poly_coords = list(zip(*polygones.exterior.coords))
                lenght = len(poly_coords[2])

                poly_coords[2] = [z_value_layer_n_1] * lenght

                x_y_z_list = []

                for x, y, z in zip(poly_coords[0], poly_coords[1], poly_coords[2]):
                    x_y_z_tuples = []

                    x_y_z_tuples.append(x)
                    x_y_z_tuples.append(y)
                    x_y_z_tuples.append(z)
                    x_y_z_list.append(x_y_z_tuples)

                coords_array = np.array(x_y_z_list)
                coords_array.reshape(3, lenght)
                self.critical_outer_overhangs_2.append(coords_array)





    # Winkel der inneren Geometrie messen
    def measure_critical_distance_inside(self, poly_inner_list):

        global z_value_layer_n, layer_n_1

        # Bisher keine 2 Winkle, macht Innen auch nicht so viel Sinn
        # kritischer Downskin-Winkel von Grad in rad
        critical_angle_1_rad = (self.critical_angle_1 * np.pi) / 180
        # critical_angle_2_rad = (self.critical_angle_2 * np.pi) / 180

        # Aus kritischem Winkel kritische Distanz berechnen

        # durch 0 teilen vermeiden:
        if np.tan(critical_angle_1_rad) == 0:
            critical_distance_1 = 10000000000000000000
        else:
            critical_distance_1 = round(self.layer_thickness / np.tan(critical_angle_1_rad), 5)
        # critical_distance_2 = round(self.layer_thickness / np.tan(critical_angle_2_rad), 5)


        for polygone_n, polygone_n_1 in zip(poly_inner_list, poly_inner_list[1:]):


            counter = 0
            critical_inner_overhang_cache = []

            # Falls die Innengeometrie beispielsweise ein Quader ist, haben die Seiten keinen Überhang, jedoch kann die
            # letzte Schicht dennoch kritisch sein:
            if polygone_n and not polygone_n_1:
                for polys in polygone_n:
                    coords_list = list(zip(*polys.exterior.coords.xy))
                    for point in coords_list:
                        points = Point(point)
                        poly_inside_distance = points.hausdorff_distance(polys)
                        if poly_inside_distance >= critical_distance_1: #and current_distance < critical_distance_2:
                            critical_inner_overhang_cache.append(polys)
                            

            # Ermittlung der Höhe der Schicht, da Shapely die z-Koordinate in Polygonen ignoriert
            for polys in polygone_n:
                coords = polys.exterior.coords
                z_value_layer_n = coords[0][2]
                break

            # Vorgehen wie bei Außerngeometrien
            while counter < len(polygone_n):

                for i in range(0, len(polygone_n_1)):
                    layer_n = polygone_n[counter]
                    layer_n_1 = polygone_n_1[i]


                    # if layer_n.type is "Polygon":
                    #     coords_layer_n = layer_n.exterior.coords
                    #     z_value_layer_n = coords_layer_n[0][2]


                    if layer_n_1.contains(layer_n):
                        do_nothing = None

                    else:

                        # Dif wird im vergleich zu den Außengeometrien berechnet, da die Richtung der
                        # kritischen Überhänge der Innengeometrie genau entgegengesetzt sind
                        dif = layer_n.difference(layer_n_1)

                        # Dif kann theoretisch auch eine Innegeometrien haben. Bisher werden diese nicht berücksichtigt

                        if dif.equals(layer_n) is False:
                            if dif.type == "Polygon" and dif.is_empty is False:

                                save_list = []
                                coords_list = list(zip(*dif.exterior.xy))

                                for point in coords_list:

                                    points = Point(point)

                                    if layer_n_1.type == "Polygon":
                                        current_distance = round(points.distance(layer_n_1), 3)

                                        if current_distance >= critical_distance_1: #and current_distance < critical_distance_2:
                                            save_list.append(True)
                                        else:
                                            save_list.append(False)


                                if True in save_list:
                                    critical_inner_overhang_cache.append(dif)

                            if dif.type == "MultiPolygon":

                                poly_support_liste = []

                                for polygone in dif:
                                    coords_list = list(zip(*polygone.exterior.coords.xy))

                                    for point in coords_list:
                                        points = Point(point)

                                        if layer_n_1.type == "Polygon":

                                            current_distance = round(points.distance(layer_n_1), 3)

                                            if current_distance >= critical_distance_1: #and current_distance < critical_distance_2:

                                                if not any(p.equals(Polygon(coords_list)) for p in poly_support_liste):
                                                    poly_support_liste.append(Polygon(coords_list))

                                multi_support = MultiPolygon(poly_support_liste)

                                if multi_support.is_empty is False:
                                    critical_inner_overhang_cache.append(dif)


                counter += 1







            # Doppelte Einträge löschen:
            read_out_cache_list = []
            for polygons in critical_inner_overhang_cache:
                if polygons.type == "MultiPolygon":
                    for polys in polygons:
                        read_out_cache_list.append(polys)
                elif polygons.type == "Polygon":
                    read_out_cache_list.append(polygons)





            counter_1 = 0
            counter_2 = 1
            counter_3 = len(read_out_cache_list)

            while counter_1 < counter_3 - 1:

                while counter_2 < counter_3:

                    intersection_polygon = read_out_cache_list[counter_1].intersection(read_out_cache_list[counter_2])

                    if not intersection_polygon.is_empty:
                        read_out_cache_list_copy = read_out_cache_list[counter_2]
                        if read_out_cache_list[counter_1] in read_out_cache_list:
                            read_out_cache_list.remove(read_out_cache_list[counter_1])

                        if read_out_cache_list_copy in read_out_cache_list:
                            read_out_cache_list.remove(read_out_cache_list_copy)


                        read_out_cache_list.append(intersection_polygon)

                        counter_1 = 0
                        counter_2 = 1
                        counter_3 = len(read_out_cache_list)

                    else:
                        counter_1 += 1
                        counter_2 += 1
                        counter_3 = len(read_out_cache_list)






            # In Numpy Array umwandeln
            read_out_read_out_cache_list = []
            for polygones in read_out_cache_list:
                if polygones.type == "MultiPolygon":
                    for polys in polygones:
                        read_out_read_out_cache_list.append(polys)
                elif polygones.type == "Polygon":
                    read_out_read_out_cache_list.append(polygones)

            for polygones in read_out_read_out_cache_list:
                poly_coords = list(zip(*polygones.exterior.coords))
                lenght = len(poly_coords[2])

                poly_coords[2] = [z_value_layer_n] * lenght

                x_y_z_list = []

                for x, y, z in zip(poly_coords[0], poly_coords[1], poly_coords[2]):
                    x_y_z_tuples = []

                    x_y_z_tuples.append(x)
                    x_y_z_tuples.append(y)
                    x_y_z_tuples.append(z)
                    x_y_z_list.append(x_y_z_tuples)

                coords_array = np.array(x_y_z_list)
                coords_array.reshape(3, lenght)
                self.critical_inner_overhangs.append(coords_array)











    # Generierung der außen liegenden Supports
    def get_support_layers(self, layer_thickness, poly_list):

        # Liste zum speichern der Supports
        self.support_layers = []

        # Liste zum zwischenspeichern
        support_cache = []

        # Maximale Z Höhe der Stützstrukturen ermitteln
        max_z = 0
        for layers in self.critical_outer_overhangs:
            if layers[0][2] > max_z:
                max_z = layers[0][2]


        # Aus kritischen Überhängen Schichten der Stützstruktur generieren
        for layers in self.critical_outer_overhangs:

            layer_height = layers[0][2]

            counter = 0
            while counter < layer_height:
                layers[:, 2] = counter
                lay_copy = copy.copy(layers)
                support_cache.append(lay_copy)
                counter += layer_thickness

        support_polygon_cache = []
        current_layer_height = 0
        while current_layer_height < max_z:

            support_polygons = []
            for outline in support_cache:

                x_values = (np.transpose(outline)[0])
                y_values = (np.transpose(outline)[1])
                z_values = (np.transpose(outline)[2])

                x_y_z_list = []

                for x, y, z in zip(x_values, y_values, z_values):
                    x_y_z_tuples = []

                    x_y_z_tuples.append(x)
                    x_y_z_tuples.append(y)
                    x_y_z_tuples.append(z)

                    x_y_z_list.append(x_y_z_tuples)

                support_polygon = Polygon(x_y_z_list)

                if z_values[0] == current_layer_height:
                    support_polygons.append(support_polygon)


            # Einzelne Schichten die in der gleichen z-Höhe liegen und sich berühren zu einer Schicht verbinden
            counter_1 = 0
            counter_2 = 1
            counter_3 = len(support_polygons)

            while counter_1 < counter_3 - 1:
                while counter_2 < counter_3:

                    union_polygon = support_polygons[counter_1].union(support_polygons[counter_2])
                    if union_polygon.type == "Polygon":

                        support_polygons_copy = support_polygons[counter_2]
                        if support_polygons[counter_1] in support_polygons:
                            support_polygons.remove(support_polygons[counter_1])

                        if support_polygons_copy in support_polygons:
                            support_polygons.remove(support_polygons_copy)

                        support_polygons.append(union_polygon)
                        counter_3 = len(support_polygons)

                        counter_1 = 0
                        counter_2 = 1
                    else:

                        counter_2 += 1
                        counter_3 = len(support_polygons)
                counter_1 += 1




            # Kollisionen mit dem Bauteil entfernen

            polygons_in_this_layer = []
            for polygons in poly_list:
                for polys in polygons:
                    coords = polys.exterior.coords
                    if coords[0][2] == round((current_layer_height), 3):
                        polygons_in_this_layer.append(polys)

            for polygons in polygons_in_this_layer:
                support_copy = copy.copy(support_polygons)
                for supports in support_copy:
                    if polygons.contains(supports):
                        support_polygons.remove(supports)
                    else:
                        dif_poly = supports.difference(polygons)
                        if not dif_poly == supports:
                            if not dif_poly.type == "MultiPolygon":
                                support_polygons.remove(supports)
                                support_polygons.append(dif_poly)
                            else:
                                support_polygons.remove(supports)
                                for polis in dif_poly:
                                    support_polygons.append(polis)


            support_polygon_cache.append(support_polygons)
            current_layer_height += layer_thickness

        # In ein mit der Bahnplanung kompatibles Numpy Array umwandeln
        for polygones in support_polygon_cache:

            for poly in polygones:

                poly_coords = list(zip(*poly.exterior.coords))
                lenght = len(poly_coords[2])

                # Wieder in Array umwandeln

                x_y_z_list = []
                extra_list = []     # Nur damit die Array Form mit der Bahnplanung funktioniert
                extra_list_2 = []   # Nur damit die Array Form mit der Bahnplanung funktioniert

                for x, y, z in zip(poly_coords[0], poly_coords[1], poly_coords[2]):
                    x_y_z_tuples = []

                    x_y_z_tuples.append(x)
                    x_y_z_tuples.append(y)
                    x_y_z_tuples.append(z)
                    x_y_z_list.append(x_y_z_tuples)

                extra_list_2.append(x_y_z_list)
                coords_array = np.array(extra_list_2)
                coords_array.reshape(3, lenght)
                extra_list.append(coords_array)
                self.support_layers.append(extra_list)



    #def delete_support_for_small_overhangs(self):
    # layer_n.almost_equals(layer_n_1)



    # Generierung der außen liegenden Supports 2
    def get_support_layers_2(self, layer_thickness, poly_list):

            # Liste zum speichern der Supports
            self.support_layers_2 = []

            # Liste zum zwischenspeichern
            support_cache = []

            # Maximale Z Höhe der Stützstrukturen ermitteln
            max_z = 0
            for layers in self.critical_outer_overhangs_2:
                if layers[0][2] > max_z:
                    max_z = layers[0][2]

            # Aus kritischen Überhängen Schichten der Stützstruktur generieren
            for layers in self.critical_outer_overhangs_2:

                layer_height = layers[0][2]

                # Schichten bis auf z=0 nach untern ziehen
                counter = 0
                while counter < layer_height:
                    layers[:, 2] = counter
                    lay_copy = copy.copy(layers)
                    support_cache.append(lay_copy)
                    counter += layer_thickness

            support_polygon_cache = []
            current_layer_height = 0
            while current_layer_height < max_z:

                support_polygons = []
                for outline in support_cache:

                    x_values = (np.transpose(outline)[0])
                    y_values = (np.transpose(outline)[1])
                    z_values = (np.transpose(outline)[2])

                    x_y_z_list = []

                    for x, y, z in zip(x_values, y_values, z_values):
                        x_y_z_tuples = []

                        x_y_z_tuples.append(x)
                        x_y_z_tuples.append(y)
                        x_y_z_tuples.append(z)

                        x_y_z_list.append(x_y_z_tuples)

                    support_polygon = Polygon(x_y_z_list)

                    if z_values[0] == current_layer_height:
                        support_polygons.append(support_polygon)

                # Einzelne Schichten die in der gleichen z-Höhe liegen und sich berühren zu einer Schicht verbinden
                counter_1 = 0
                counter_2 = 1
                counter_3 = len(support_polygons)

                while counter_1 < counter_3 - 1:
                    while counter_2 < counter_3:

                        union_polygon = support_polygons[counter_1].union(support_polygons[counter_2])
                        if union_polygon.type == "Polygon":

                            support_polygons_copy = support_polygons[counter_2]
                            if support_polygons[counter_1] in support_polygons:
                                support_polygons.remove(support_polygons[counter_1])

                            if support_polygons_copy in support_polygons:
                                support_polygons.remove(support_polygons_copy)

                            support_polygons.append(union_polygon)
                            counter_3 = len(support_polygons)

                            counter_1 = 0
                            counter_2 = 1
                        else:

                            counter_2 += 1
                            counter_3 = len(support_polygons)
                    counter_1 += 1

                # Kollisionen mit dem Bauteil entfernen

                polygons_in_this_layer = []
                for polygons in poly_list:
                    for polys in polygons:
                        coords = polys.exterior.coords
                        if coords[0][2] == round((current_layer_height), 3):
                            polygons_in_this_layer.append(polys)

                for polygons in polygons_in_this_layer:
                    support_copy = copy.copy(support_polygons)
                    for supports in support_copy:
                        if polygons.contains(supports):
                            support_polygons.remove(supports)
                        else:
                            dif_poly = supports.difference(polygons)
                            if not dif_poly == supports:
                                if not dif_poly.type == "MultiPolygon":
                                    support_polygons.remove(supports)
                                    support_polygons.append(dif_poly)
                                else:
                                    support_polygons.remove(supports)
                                    for polis in dif_poly:
                                        support_polygons.append(polis)

                support_polygon_cache.append(support_polygons)
                current_layer_height += layer_thickness

            # In ein mit der Bahnplanung kompatibles Numpy Array umwandeln
            for polygones in support_polygon_cache:

                for poly in polygones:

                    poly_coords = list(zip(*poly.exterior.coords))
                    lenght = len(poly_coords[2])

                    # Wieder in Array umwandeln

                    x_y_z_list = []
                    extra_list = []  # Nur damit die Array Form mit der Bahnplanung funktioniert
                    extra_list_2 = []  # Nur damit die Array Form mit der Bahnplanung funktioniert

                    for x, y, z in zip(poly_coords[0], poly_coords[1], poly_coords[2]):
                        x_y_z_tuples = []

                        x_y_z_tuples.append(x)
                        x_y_z_tuples.append(y)
                        x_y_z_tuples.append(z)
                        x_y_z_list.append(x_y_z_tuples)

                    extra_list_2.append(x_y_z_list)
                    coords_array = np.array(extra_list_2)
                    coords_array.reshape(3, lenght)
                    extra_list.append(coords_array)
                    self.support_layers_2.append(extra_list)











    # Generierung der innen liegenden Supports

    def get_inner_support_layers(self, layer_thickness, poly_inner_list):

        self.inner_support_layers = []

        support_cache = []

        # Maximale z-Höhe der Supports ermitteln
        max_z = 0
        for layers in self.critical_inner_overhangs:
            if layers[0][2] > max_z:
                max_z = layers[0][2]

        # Supports aus den kritischen Überhängen generieren
        for layers in self.critical_inner_overhangs:

            layer_height = layers[0][2]

            counter = 0
            while counter < layer_height:
                layers[:, 2] = counter
                lay_copy = copy.copy(layers)
                support_cache.append(lay_copy)
                counter += layer_thickness

        support_polygon_cache = []
        current_layer_height = 0
        while current_layer_height < max_z:

            support_polygons = []
            for outline in support_cache:

                x_values = (np.transpose(outline)[0])
                y_values = (np.transpose(outline)[1])
                z_values = (np.transpose(outline)[2])

                x_y_z_list = []

                for x, y, z in zip(x_values, y_values, z_values):
                    x_y_z_tuples = []

                    x_y_z_tuples.append(x)
                    x_y_z_tuples.append(y)
                    x_y_z_tuples.append(z)

                    x_y_z_list.append(x_y_z_tuples)

                support_polygon = Polygon(x_y_z_list)

                if z_values[0] == current_layer_height:
                    support_polygons.append(support_polygon)



            # Einzelne Schichten die in der gleichen z-Höhe liegen und sich berühren zu einer Schicht verbinden
            counter_1 = 0
            counter_2 = 1
            counter_3 = len(support_polygons)

            while counter_1 < counter_3 - 1:
                while counter_2 < counter_3:

                    union_polygon = support_polygons[counter_1].union(support_polygons[counter_2])
                    if union_polygon.type == "Polygon":

                        support_polygons_copy = support_polygons[counter_2]
                        if support_polygons[counter_1] in support_polygons:
                            support_polygons.remove(support_polygons[counter_1])

                        if support_polygons_copy in support_polygons:
                            support_polygons.remove(support_polygons_copy)

                        support_polygons.append(union_polygon)
                        counter_3 = len(support_polygons)

                        counter_1 = 0
                        counter_2 = 1

                    else:


                        counter_2 += 1
                        counter_3 = len(support_polygons)
                counter_1 += 1

            # ALTE METHODE
            # for polygons in polygons_in_this_layer:
            #     support_copy = copy.copy(support_polygons)
            #     for supports in support_copy:
            #         if polygons.contains(supports):
            #             support_polygons.remove(supports)
            #         else:
            #             dif_poly = supports.difference(polygons)
            #             if not dif_poly == supports:
            #                 if not dif_poly.type == "MultiPolygon":
            #                     support_polygons.remove(supports)
            #                     support_polygons.append(dif_poly)
            #                 else:
            #                     support_polygons.remove(supports)
            #                     for polis in dif_poly:
            #                         support_polygons.append(polis)



            # NEUE METHODE:
            inner_polygons_in_this_layer = []
            for polygons in poly_inner_list:
                for polys in polygons:
                    coords = polys.exterior.coords
                    if coords[0][2] == round((current_layer_height),3):
                        inner_polygons_in_this_layer.append(polys)


            for polys in inner_polygons_in_this_layer:
                support_copy = copy.copy(support_polygons)
                for supports in support_copy:
                    intersection_poly = supports.intersection(polys)
                    if supports.contains(polys):
                        support_polygons.remove(supports)
                        support_polygons.append(polys)

                    # Der Teil sollte mal getestet werden

                    elif not intersection_poly.equals(supports):
                        if intersection_poly.type == "Polygon":
                            support_polygons.remove(supports)
                            support_polygons.append(intersection_poly)
                        else:
                            support_polygons.remove(supports)
                            for polis in intersection_poly:
                                support_polygons.append(polis)

            # Wenn in der Schicht keine Innengeometrie ist, kann dort auch kein Support sein
            if not inner_polygons_in_this_layer:
                support_polygons = []


            support_polygon_cache.append(support_polygons)


            current_layer_height += layer_thickness

        # In ein mit der Bahnplanung kompatibles Numpy Array umwandeln
        for polygones in support_polygon_cache:

            for poly in polygones:

                poly_coords = list(zip(*poly.exterior.coords))
                lenght = len(poly_coords[2])

                # Wieder in Array umwandeln

                x_y_z_list = []
                extra_list = []  # Damit es in der gleichen Array-Form ist, wie das Array des Slicers
                extra_list_2 = []

                for x, y, z in zip(poly_coords[0], poly_coords[1], poly_coords[2]):
                    x_y_z_tuples = []

                    x_y_z_tuples.append(x)
                    x_y_z_tuples.append(y)
                    x_y_z_tuples.append(z)
                    x_y_z_list.append(x_y_z_tuples)

                extra_list_2.append(x_y_z_list)
                coords_array = np.array(extra_list_2)
                coords_array.reshape(3, lenght)
                extra_list.append(coords_array)
                self.inner_support_layers.append(extra_list)


    def delete_small_overhangs(self, pass_value, first_layer, stop_pass_value):

        support_list = []
        for supports in self.support_layers_2:
            for outline in supports:

                    x_values = (np.transpose(outline)[0])
                    y_values = (np.transpose(outline)[1])
                    z_values = (np.transpose(outline)[2])

                    x_y_z_list = []

                    for x, y, z in zip(x_values, y_values, z_values):
                        x_y_z_tuples = []

                        x_y_z_tuples.append(x)
                        x_y_z_tuples.append(y)
                        x_y_z_tuples.append(z)

                        x_y_z_list.append(x_y_z_tuples)

                    support_polygon = Polygon(x_y_z_list)
                    support_list.append(support_polygon)

        support_delete_list = []
        max_distance_list = []
        for layer_n1, layer_n2, layer_n3 in zip(support_list, support_list[1:], support_list[2:]):
            max_distance_list.append(layer_n1.hausdorff_distance(layer_n2))
            if layer_n1.almost_equals(layer_n2) and layer_n2.almost_equals(layer_n3):
                support_delete_list.append(False)
            else:
                support_delete_list.append(True)

        support_delete_list_2 = []
        for values in max_distance_list:
            if values > pass_value:
                support_delete_list_2.append(False)
            else:
                support_delete_list_2.append(True)

        for points in first_layer:

            x_values = (np.transpose(points)[0])
            y_values = (np.transpose(points)[1])
            z_values = (np.transpose(points)[2])

            x_y_z_list = []

            for x, y, z in zip(x_values, y_values, z_values):
                x_y_z_tuples = []

                x_y_z_tuples.append(x)
                x_y_z_tuples.append(y)
                x_y_z_tuples.append(z)

                x_y_z_list.append(x_y_z_tuples)

            polygon_first_layer = Polygon(x_y_z_list)

        max_distance_list_2 = []
        coords_list = list(zip(*support_list[0].exterior.xy))
        for point in coords_list:
            points = Point(point)
            max_distance_list_2.append(polygon_first_layer.distance(points))


        support_delete_list_3 = []
        for values in max_distance_list_2:
            if values > stop_pass_value:
                support_delete_list_3.append(True)


        if True in support_delete_list and not False in support_delete_list_2 and not True in support_delete_list_3:
            self.support_layers_2 = []






        # Das gleiche noch für die zweite Support Schicht
       # for self.support_layer:


    # Support-Daten in Datenbank speichern
    def save_support_data(self, model_name):

        save_data = {'support_data': self.support_layers, 'support_data_2' : self.support_layers_2, 'inner_support_data': self.inner_support_layers}

        with open(r'data_numpy/' + model_name + '_support_data' + '.pickle', 'wb') as f:
            pickle.dump(save_data, f)




    # Kritische Überhänge plotten
    def plot_overhangs(self, slice_of_all_layer_points):

        fig_5 = plt.figure()
        ax = mplot3d.Axes3D(fig_5)
        ax.set_title("Model")
        ax.set_xlabel('X [mm]')
        ax.set_ylabel('Y [mm]')
        ax.set_zlabel('Z [mm]')
        # scale = Model.model.points.flatten('F')
        # ax.auto_scale_xyz(scale, scale, scale)

        for shells_of_layer in slice_of_all_layer_points:
            for shell in shells_of_layer:
                for outline in shell:
                    ax.plot(np.transpose(outline)[0], np.transpose(outline)[1], np.transpose(outline)[2], color="b", alpha=0.3)

        for shell in self.critical_outer_overhangs:
            ax.plot(np.transpose(shell)[0], np.transpose(shell)[1], np.transpose(shell)[2], color="yellow", alpha = 1.0)

        for shell in self.critical_inner_overhangs:
            ax.plot(np.transpose(shell)[0], np.transpose(shell)[1], np.transpose(shell)[2], color="red")

        for shell in self.critical_outer_overhangs_2:
            ax.plot(np.transpose(shell)[0], np.transpose(shell)[1], np.transpose(shell)[2], color="red")

        # TESTPLOTS
        # black_circle = Point(2.5, 2.5).buffer(1.418, 9)
        # x,y = black_circle.exterior.coords.xy
        # z = [3.90]*len(x)
        # ax.plot(x, y, z, color="black")
        #ax.plot([2.43, 2.43], [0, 5], [3.06, 3.06], color="black")

        for shell_of_layers in self.support_layers:
            for shell in shell_of_layers:
                for outline in shell:
                    ax.plot(np.transpose(outline)[0], np.transpose(outline)[1], np.transpose(outline)[2], color="grey", alpha = 0.5)

        for shell_of_layers in self.support_layers_2:
            for shell in shell_of_layers:
                for outline in shell:
                    ax.plot(np.transpose(outline)[0], np.transpose(outline)[1], np.transpose(outline)[2], color="black", alpha = 0.5)

        for shell_of_layers in self.inner_support_layers:
            for shell in shell_of_layers:
                for outline in shell:
                    ax.plot(np.transpose(outline)[0], np.transpose(outline)[1], np.transpose(outline)[2], color="grey", alpha = 1)

        plt.show()




    # Gespeicherten Support öffnen und anzeigen (zur Überprüfung)
    def open_support_data(self, model_name):



        with open(r'./data_numpy/' + model_name + '_support_data.pickle', 'rb') as f:
            support_data = pickle.load(f)
        support = support_data['support_data']
        support_2 = support_data['support_data_2']
        inner_support = support_data['inner_support_data']

        fig_5 = plt.figure()
        ax = mplot3d.Axes3D(fig_5)
        ax.set_title("Model")
        ax.set_xlabel('X [mm]')
        ax.set_ylabel('Y [mm]')
        ax.set_zlabel('Z [mm]')
        # scale = Model.model.points.flatten('F')
        # ax.auto_scale_xyz(scale, scale, scale)

        for shell_of_layers in support:
            for shell in shell_of_layers:
                for outline in shell:
                    ax.plot(np.transpose(outline)[0], np.transpose(outline)[1], np.transpose(outline)[2], color="grey",
                            alpha=0.3)

        for shell_of_layers in support_2:
            for shell in shell_of_layers:
                for outline in shell:
                    ax.plot(np.transpose(outline)[0], np.transpose(outline)[1], np.transpose(outline)[2], color="black",
                            alpha=0.3)

        for shell_of_layers in inner_support:
            for shell in shell_of_layers:
                for outline in shell:
                    ax.plot(np.transpose(outline)[0], np.transpose(outline)[1], np.transpose(outline)[2], color="grey",
                            alpha=0.3)
        plt.show()





class BahnplanungSupport:

  #  def __init__(self):

    def get_column_support(self, support_layers, column_distance=1, column_diameter=0.1, hatch_offset_column=0.1, layer_thickness=0.01, maximum_z=100):



        auto_st_hatches_of_all_layer = []

        ordered_support_layers = []

        # Toleranz um kleinere Rundungsfehler auszugleichen
        tolerance = 0.001
        layer_height = 0
        while layer_height <= maximum_z:
            polis_in_this_height = []
            for layer in support_layers:
                for shell in layer:
                    for outline in shell:
                        if outline[0][2] >= layer_height-tolerance and outline[0][2] <= layer_height+tolerance:
                            polis_in_this_height.append(outline)

            ordered_support_layers.append(polis_in_this_height)
            layer_height += layer_thickness

        for layers in ordered_support_layers:

            circle_cache = []
            polys_in_this_layer = []
            for shell in layers:

                    # x- ,y- ,z-Werte durch transponieren des Numpy Array
                    x_values = (np.transpose(shell)[0])
                    y_values = (np.transpose(shell)[1])
                    z_values = (np.transpose(shell)[2])

                    x_y_z_list = []

                    # In für Shapely-kompatibles Format formatieren
                    for x, y, z in zip(x_values, y_values, z_values):
                        x_y_z_tuples = []

                        x_y_z_tuples.append((round(x, 3)))
                        x_y_z_tuples.append((round(y, 3)))
                        x_y_z_tuples.append((round(z, 3)))

                        x_y_z_list.append(x_y_z_tuples)

                    if len(x_y_z_list) >= 3:

                        # Shapely-Polygon erstellen
                        outer_polygon = Polygon(x_y_z_list)

                        # Prüfen, ob das Polygon richtig erstellt wurde
                        if outer_polygon.is_valid:
                            polys_in_this_layer.append(outer_polygon)
                        else:
                            print("FEHLER_BAHNPLANUNG_SUPPORT")
                    else:
                        print("FEHLER_BAHNPLANUNG_SUPPORT")



            for polis in polys_in_this_layer:
                if polis.type == "Polygon":
                    bounds = polis.bounds
                    xmin = bounds[0]
                    xmax = bounds[2]+1
                    ymin = bounds[1]
                    ymax = bounds[3]+1

                    i_x = column_diameter #0-(column_diameter/2)
                    i_y = column_diameter #0-(column_diameter/2)


                    while i_x < xmax:

                        while i_y < ymax:

                            d = column_diameter
                            while d > 0:
                                circle = Point(i_x, i_y).buffer(d, 3)
                                if polis.contains(circle):
                                    circle_coords = list(zip(*circle.exterior.coords))
                                    x_y_list = []
                                    for x, y in zip(circle_coords[0], circle_coords[1]):
                                        x_y_tuples = []

                                        x_y_tuples.append(x)
                                        x_y_tuples.append(y)

                                        x_y_list.append(x_y_tuples)

                                    circle_coords_array = np.array(x_y_list)
                                    circle_coords_array.reshape(2, len(circle_coords[0]))
                                    circle_cache.append(circle_coords_array)

                                elif polis.intersects(circle):
                                    new_circle = circle.intersection(polis)
                                    new_circle_coords = list(zip(*new_circle.exterior.coords))
                                    x_y_list = []
                                    for x, y in zip(new_circle_coords[0], new_circle_coords[1]):
                                        x_y_tuples = []

                                        x_y_tuples.append(x)
                                        x_y_tuples.append(y)

                                        x_y_list.append(x_y_tuples)

                                    circle_coords_array = np.array(x_y_list)
                                    circle_coords_array.reshape(2, len(new_circle_coords[0]))
                                    circle_cache.append(circle_coords_array)

                                d = d-2*hatch_offset_column

                            i_y+=column_distance

                        i_x+=column_distance
                        i_y = 0


            auto_st_hatches_of_all_layer.append(circle_cache)
        return auto_st_hatches_of_all_layer



    def get_wall_support(self, support_layers, wall_distance = 1, number_of_hatches = 3, hatch_offset_wall = 0.1, layer_thickness = 0.03, maximum_z = 100):



        auto_st_hatches_of_all_layer = []

        ordered_support_layers = []


        layer_height = 0
        while layer_height <= maximum_z:
            polis_in_this_height = []
            for layer in support_layers:
                for shell in layer:
                    for outline in shell:
                        if outline[0][2] == layer_height:
                            polis_in_this_height.append(outline)

            ordered_support_layers.append(polis_in_this_height)
            layer_height += layer_thickness



        layer_counter = 0
        for layers in ordered_support_layers:

            line_cache = []
            polis_in_this_layer = []
            for shell in layers:

                    # x- ,y- ,z-Werte durch transponieren des Numpy Array
                    x_values = (np.transpose(shell)[0])
                    y_values = (np.transpose(shell)[1])
                    z_values = (np.transpose(shell)[2])

                    x_y_z_list = []

                    # In für Shapely-kompatibles Format formatieren
                    for x, y, z in zip(x_values, y_values, z_values):
                        x_y_z_tuples = []

                        x_y_z_tuples.append((round(x, 3)))
                        x_y_z_tuples.append((round(y, 3)))
                        x_y_z_tuples.append((round(z, 3)))

                        x_y_z_list.append(x_y_z_tuples)

                    if len(x_y_z_list) >= 3:

                        # Shapely-Polygon erstellen
                        outer_polygon = Polygon(x_y_z_list)

                        # Prüfen, ob das Polygon richtig erstellt wurde
                        if outer_polygon.is_valid:
                            polis_in_this_layer.append(outer_polygon)
                        else:
                            print("FEHLER_BAHNPLANUNG_SUPPORT")
                    else:
                        print("FEHLER_BAHNPLANUNG_SUPPORT")



            for polis in polis_in_this_layer:
                if polis.type == "Polygon":
                    bounds = polis.bounds
                    xmin = bounds[0]
                    xmax = bounds[2]
                    ymin = bounds[1]
                    ymax = bounds[3]


                    i_y = 0



                    while i_y < ymax:

                        for i in range(0, number_of_hatches):

                            # Die Richtung der Scanlinie wird jede Schicht um 180° gedreht.
                            # Dafür wird der Definition der Scanlinien bei geraden und ungeraden Schichten umgedreht
                            if layer_counter%2:
                                line = LineString([(0, i_y+i*hatch_offset_wall),(5000, i_y+i*hatch_offset_wall)])
                            else:
                                line = LineString([(5000, i_y + i * hatch_offset_wall), (0, i_y + i * hatch_offset_wall)])

                            line_intersection = line.difference(polis)
                            inner_line = line.difference(line_intersection)

                            line_coords = list(zip(*inner_line.coords))
                            #First line does not intercept with Polygon --> if statement
                            if line_coords != []:
                                x_y_list = []
                                for x, y in zip(line_coords[0], line_coords[1]):
                                    x_y_tuples = []

                                    x_y_tuples.append(x)
                                    x_y_tuples.append(y)

                                    x_y_list.append(x_y_tuples)


                                line_coords_array = np.array(x_y_list)

                                line_cache.append(line_coords_array)


                        i_y+=wall_distance
            layer_counter+=1

            auto_st_hatches_of_all_layer.append(line_cache)



        return auto_st_hatches_of_all_layer



    def get_lattice_support(self, support_layers, lattice_distance = 1, number_of_hatches = 3, hatch_offset_lattice = 0.1, layer_thickness = 0.03, maximum_z = 100):

            auto_st_hatches_of_all_layer = []

            ordered_support_layers = []

            layer_height = 0
            while layer_height <= maximum_z:
                polis_in_this_height = []
                for layer in support_layers:
                    for shell in layer:
                        for outline in shell:
                            if outline[0][2] == layer_height:
                                polis_in_this_height.append(outline)

                ordered_support_layers.append(polis_in_this_height)
                layer_height += layer_thickness

            layer_counter = 0
            for layers in ordered_support_layers:

                line_cache = []
                polis_in_this_layer = []
                for shell in layers:

                        # x- ,y- ,z-Werte durch transponieren des Numpy Array
                        x_values = (np.transpose(shell)[0])
                        y_values = (np.transpose(shell)[1])
                        z_values = (np.transpose(shell)[2])

                        x_y_z_list = []

                        # In für Shapely-kompatibles Format formatieren
                        for x, y, z in zip(x_values, y_values, z_values):
                            x_y_z_tuples = []

                            x_y_z_tuples.append((round(x, 3)))
                            x_y_z_tuples.append((round(y, 3)))
                            x_y_z_tuples.append((round(z, 3)))

                            x_y_z_list.append(x_y_z_tuples)

                        if len(x_y_z_list) >= 3:

                            # Shapely-Polygon erstellen
                            outer_polygon = Polygon(x_y_z_list)

                            # Prüfen, ob das Polygon richtig erstellt wurde
                            if outer_polygon.is_valid:
                                polis_in_this_layer.append(outer_polygon)
                            else:
                                print("FEHLER_BAHNPLANUNG_SUPPORT")
                        else:
                            print("FEHLER_BAHNPLANUNG_SUPPORT")



                for polis in polis_in_this_layer:
                    if polis.type == "Polygon":
                        bounds = polis.bounds
                        xmin = bounds[0]
                        xmax = bounds[2]
                        ymin = bounds[1]
                        ymax = bounds[3]


                        i_y = 0
                        i_x = 0


                        while i_y < ymax:

                            for i in range(1, number_of_hatches):

                                # Die Richtung der Scanlinie wird jede Schicht um 180° gedreht.
                                # Dafür wird der Definition der Scanlinien bei geraden und ungeraden Schichten umgedreht
                                if layer_counter%2:
                                    line = LineString([(0, i_y+i*hatch_offset_lattice),(bounds[2], i_y+i*hatch_offset_lattice)])
                                else:
                                    line = LineString([(bounds[2], i_y + i * hatch_offset_lattice), (0, i_y + i * hatch_offset_lattice)])

                                #line_intersection: wo die Stützstruktur mit dem Modell zusammentrifft. Da soll die stützstruktur aufhören
                                line_intersection = line.difference(polis)
                                inner_line = line.difference(line_intersection)


                                line_coords = list(zip(*inner_line.coords))
                                if line_coords != []:
                                    x_y_list = []
                                    for x, y in zip(line_coords[0], line_coords[1]):
                                        x_y_tuples = []

                                        x_y_tuples.append(x)
                                        x_y_tuples.append(y)

                                        x_y_list.append(x_y_tuples)


                                    line_coords_array = np.array(x_y_list)

                                    line_cache.append(line_coords_array)


                            i_y+=lattice_distance

                        while i_x < xmax:

                            for i_2 in range(0, number_of_hatches):

                                if layer_counter%2:
                                    line_2 = LineString([((i_x+i_2*hatch_offset_lattice), 0), ((i_x + i_2 * hatch_offset_lattice), 5000)])
                                else:
                                    line_2 = LineString([((i_x + i_2 * hatch_offset_lattice), 5000), ((i_x + i_2 * hatch_offset_lattice), 0)])

                                line_2_intersection = line_2.difference(polis)
                                inner_line_2 = line_2.difference(line_2_intersection)

                                line_2_coords = list(zip(*inner_line_2.coords))

                                if line_2_coords:
                                    x_y_list = []
                                    for x, y in zip(line_2_coords[0], line_2_coords[1]):
                                        x_y_tuples = []

                                        x_y_tuples.append(x)
                                        x_y_tuples.append(y)

                                        x_y_list.append(x_y_tuples)

                                    line_2_coords_array = np.array(x_y_list)

                                    line_cache.append(line_2_coords_array)

                            i_x += lattice_distance


                layer_counter+=1
                auto_st_hatches_of_all_layer.append(line_cache)
            return auto_st_hatches_of_all_layer





""" #################### Ausführen des Skript Support ohne Skript Execute #################### """


"""Oeffnen des geslicten Modells"""

# with open(r'./data_numpy/F_points_of_layers.pickle', 'rb') as f:
#     slices_of_all_layers = pickle.load(f)
# slices_of_all_layers = slices_of_all_layers['slice_data']
#
# slices_of_all_layers_copy = copy.copy(slices_of_all_layers)
#
#
#
#
# """Eingabefeld"""
#
# layer_thickness = 0.1       # Wird im Execute_slicer automatisch übernommen
# critical_angle = 45
# model_name = "testgeometrie"     # Wird im Execute_slicer automatisch übernommen
#
#
# """Code für die Ausführung"""
#
# get_coords = GetCoords(slices_of_all_layers_copy)
# poly_list = get_coords.get_outer_shells()
# poly_inner_list = get_coords.get_inner_shells()
#
#
# angle = Angle_Analysis(critical_angle, layer_thickness)
# angle.measure_critical_distance_outside(poly_list)
# angle.measure_critical_distance_inside(poly_inner_list)
# angle.get_support_layers(layer_thickness, poly_list)
# angle.get_inner_support_layers(layer_thickness, poly_inner_list)
# angle.plot_overhangs(slices_of_all_layers_copy)
# angle.save_support_data(model_name)
# angle.open_support_data(model_name)








