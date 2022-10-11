FROM public.ecr.aws/lambda/python:3.7
COPY . .
RUN pip install geopandas==0.9.0 &&\
    #pip install osmnx==1.2.2 &&\
    pip install osmnx &&\
    pip install matplotlib==3.5.2 &&\
    pip install scipy==1.7.1 &&\
    pip install shapely==1.7.1 &&\
    #pip install pandas==1.4.3 &&\
    pip install pandas &&\
    #pip install networkx==2.8.4 &&\
    pip install networkx &&\
    #pip install numpy==1.22.4
    pip install numpy
CMD [ "path.pathfinder" ]
