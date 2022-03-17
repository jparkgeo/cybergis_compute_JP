# Import necessary packages
import geopandas as gpd
import osmnx as ox
import networkx as nx
from tqdm import tqdm
import time

start_time = time.time()
# This function helps you to find the nearest OSM node from a given GeoDataFrame
# If geom type is point, it will take it without modification, but
# IF geom type is polygon or multipolygon, it will take its centroid to calculate the nearest element.
def find_nearest_osm(network, gdf):
    for idx, row in tqdm(gdf.iterrows(), total=gdf.shape[0]):
        if row.geometry.geom_type == 'Point':
            nearest_osm = ox.distance.nearest_nodes(network,
                                                    X=row.geometry.x,
                                                    Y=row.geometry.y
                                                   )
        elif row.geometry.geom_type == 'Polygon' or row.geometry.geom_type == 'MultiPolygon':
            nearest_osm = ox.distance.nearest_nodes(network,
                                        X=row.geometry.centroid.x,
                                        Y=row.geometry.centroid.y
                                       )
        else:
            print(row.geometry.geom_type)
            continue

        gdf.at[idx, 'nearest_osm'] = nearest_osm

    return gdf


# Healthcare resources
hc = gpd.read_file('./data/healthcare.shp')
hc.head()

# Census block groups
cbg = gpd.read_file('./data/census_block_group.shp')
cbg.head()

hc = hc.to_crs(epsg=26971)
cbg = cbg.to_crs(epsg=26971)

travel_time = 10
dist = 30 * travel_time / 60
dist = dist * 1.6 * 1000 # Translate to Meter per Hour to match the unit with the coordinates system (epsg 26971)
print(f'{dist} meter is the threshold distance can travel within {travel_time} minutes.')

G = ox.load_graphml('./data/champaign_osm.graphml')
# G = ox.graph_from_place('Champaign County, IL, USA', network_type='drive', simplify=True)
G = ox.projection.project_graph(G, to_crs='epsg:26971')

hc = find_nearest_osm(G, hc)
cbg = find_nearest_osm(G, cbg)
nodes, edges = ox.graph_to_gdfs(G, nodes=True, edges=True, node_geometry=True)

convex_hulls = gpd.GeoSeries()
for idx, row in tqdm(hc.iterrows(), total=hc.shape[0]):
    temp_nodes = nx.single_source_dijkstra_path_length(G, row['nearest_osm'], dist, weight='length')
    access_nodes = nodes.loc[nodes.index.isin(temp_nodes.keys()), 'geometry']
    access_nodes_ = gpd.GeoSeries(access_nodes.unary_union.convex_hull)
    convex_hulls = convex_hulls.append(access_nodes_, ignore_index=True)

convex_hulls_union = convex_hulls.unary_union
convex_hulls_union = gpd.GeoSeries(convex_hulls_union)

y_convex_hull = cbg.loc[cbg.geometry.centroid.within(convex_hulls_union[0])]
n_convex_hull = cbg.loc[~cbg.geometry.centroid.within(convex_hulls_union[0])]

y_convex_hull.to_file('./results/within_hs.shp')
n_convex_hull.to_file('./results/n_within_hs.shp')
print("--- %s seconds ---" % (time.time() - start_time))