# Autor: Sol Represa
# Fecha: 15/05/2018
# 
#
# Objetivo: construir mapa donde visualizar la intensidad de señal a partir de file gpx y csv
#install.packages("plotKML")
library(XML)
library(OpenStreetMap)
library(lubridate)
library(ggplot2)
library(ggmap)
library(rgdal)
library(maptools)
library(leaflet)
library(tidyverse)
library(RColorBrewer)
library(plotKML)

##################################

# 1. Unificar base de datos  #####

##################################


# 1.1 Abrir base de datos de señal ####

rssi <- read.csv("trace_log_from_1525892412982_to_1525895904974.csv", sep = ";", dec = ".")
                  
rssi$Timestamp <- as.POSIXct(paste(substring(rssi$Timestamp, 1,8), 
                                   substring(as.character(rssi$Timestamp), 14,23)), 
                             format = "%H:%M:%S %d/%m/%Y" )

# 1.2 Abrir base de datos GPS ####

file <- readGPX("2018-05-09_16-36_Wed.gpx", metadata = TRUE, bounds = TRUE, 
                waypoints = TRUE, tracks = TRUE, routes = TRUE)   #libreria plotKML

file <- file$tracks[[1]][[1]]

file$time <- as.POSIXct(paste(substring(file$time, 1,10), 
                              substring(file$time, 12,19), sep =" "), 
                        format = "%Y-%m-%d %H:%M:%S")


# 1.3 Elaborar tabla para hacer match en fechas con 30 seg de diferencia ####

file_rssi <- data.frame()

for (i in 1: nrow(file)){                  
  eq_30 <- which(abs(rssi$Timestamp - file[i,]$time) <= 30)   # <---- busco diferencia en menos de 30 second
  tabla_30 <- rssi[eq_30,]
  if(dim(tabla_30)[1] == 0){                                          ## si no tengo datos dentro de lo 30 minutos, coloca un NA
    salida <- data.frame(file[i,1:4],NA, NA)
  }else{
    salida <- data.frame(file[i,1:4], aggregate(rssi~Gateway, FUN=mean, data = tabla_30))
  }
  names(salida) <- c("lon", "lat", "ele", "time", "Gateway", "rssi")
  file_rssi <- rbind(file_rssi , salida)
}

rm(salida, file, rssi, tabla_30, eq_30, i)


############################################

# 2. Exportar spatial point data frame  ####

############################################

# 2.1 Generar Spatial point data frame ####

file_rssi$time <- as.character(file_rssi$time) #es necesario cambiar formato para guardarlo
myProjection <- "+proj=longlat +datum=WGS84 +no_defs +ellps=WGS84 +towgs84=0,0,0"
file_rssi.spdf <- SpatialPointsDataFrame(file_rssi[,1:2], # las coordenadas
                                         file_rssi[,3:6], # el data.frame
                                         proj4string = CRS(myProjection)) #la proyeccion

## 2.2 Guardar Shapefile ####

writeSpatialShape(file_rssi.spdf, "RSSI")   # .shp

## 2.3 Guardar kml ####

writeOGR(file_rssi.spdf, "file_rssi_ll.kml", layer="rsi", driver="KML")  # .kml





#########################################

## 3. Armar mapa estatico con ggplot

########################################


map <- get_map(location= c(lat=-34.921, lon=-57.941), maptype="roadmap", zoom=15)
ggmap(map) + geom_point(aes(x=lon, y=lat, colour=rssi, shape = Gateway), data=file_rssi, 
                        alpha=.9, size = 5) +
  ggtitle("") +
  labs(x= "Longitud", y= "Latitud") 




###########################################

# 4. Armar mapa interactivo con Leaflet ####

############################################

# Crear paleta de colores para los niveles de los factores 

#display.brewer.all()
pal <- colorFactor(brewer.pal(length(levels(file_rssi$Gateway)), "Set1"), 
                   domain = levels(file_rssi$Gateway)) 

leaflet(data=file_rssi) %>%
  addTiles() %>%  # Add default OpenStreetMap map tiles
  # addProviderTiles(providers$CartoDB.Positron) %>%
  addCircleMarkers(
    popup = ~as.character(Gateway), 
    label = ~as.character(rssi),
    radius = ~abs(rssi)/10,
    color = ~pal(Gateway),
    stroke = FALSE, fillOpacity = 0.5
  )




###########################################################################

# Otra forma de abrir .gpx 
# pero no es util para acceder a la base de datos

file <- readOGR("2018-05-09_16-36_Wed.gpx", layer = "track_points")
crd.gpx <- data.frame(file@coords)
names(crd.gpx) <- c("Long", "Lat")
