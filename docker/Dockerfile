# select base image
FROM ubuntu:19.10

# Dockerfile authors / maintainers
LABEL authors='[{"name":"Anand Bhasakaran","email":"anand.bhaskaran@epfl.ch"},\
{"name":"Daniele Bissoli","email":"daniele.bissoli@studenti.unitn.it"}]'

# set timezone for the server
ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# update application repository list and install required files
RUN apt-get update && apt-get install -y g++ python3 python3-pip libglib2.0-dev\
                                         curl freeglut3 freeglut3-dev cmake\
                                         build-essential unzip less nano wget\
                                         python3-tk screen python3-pydot graphviz\
                                         libgraphviz-dev graphviz-dev pkg-config time\
                                         libgl1-mesa-glx libgl1-mesa-dri mesa-utils\
                                         mesa-opencl-icd mesa-va-drivers mesa-vdpau-drivers \
                                         doxygen ccache libblas-dev liblapack-dev ffmpeg x264
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

# install a recent version of cmake
RUN mkdir -p cmake_src && wget -qO- https://github.com/Kitware/CMake/archive/v3.15.5.tar.gz | tar -zxvf - -C cmake_src --strip 1
RUN cd cmake_src && ./bootstrap && make -j`grep -c ^processor /proc/cpuinfo` && make install
RUN rm -r cmake_src

# header only library required to run the experiments
RUN mkdir eigen3 && wget -qO- http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz | tar -zxvf - -C eigen3 --strip 1
RUN cd eigen3 && cmake -S . -B build -DOpenGL_GL_PREFERENCE=GLVND && cmake --install build
RUN rm -r eigen3

# install a font that can be used for plotting
RUN wget https://github.com/adobe-fonts/source-sans-pro/releases/download/3.006R/source-sans-pro-3.006R.zip
RUN mkdir -p /usr/share/fonts/truetype/SourceSansPro
RUN unzip source-sans-pro-3.006R.zip && mv source-sans-pro-3.006R/TTF/* /usr/share/fonts/truetype/SourceSansPro
RUN fc-cache -f -v

# Python configuration and dependencies installation
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

COPY requirements.txt .
RUN pip3 install --upgrade pip
RUN pip3 install -r requirements.txt && rm requirements.txt

# this alias is just for simplifying the navigation within the container
RUN echo 'alias ll="ls -lh"' >> /etc/bash.bashrc

WORKDIR /home/tensoft

# prepare the container to start the bash shell once launched
ENV TERM=xterm
CMD ["/bin/bash"]
