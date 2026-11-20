# UTMPoint.msg
#
# Represents a point in UTM coordinates

# UTM coordinates
float64 northing   # UTM northing in meters
float64 easting    # UTM easting in meters

# UTM Zone and Hemisphere
uint8 zone         # UTM zone
bool north         # UTM hemisphere (north=true, south=false)
