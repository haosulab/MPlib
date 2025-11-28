## build docker (only for the first time)
```bash
sudo docker build -f docker/Dockerfile -t mplib-manylinux2014 .

# with proxy (for china mainland)
sudo docker build -f docker/Dockerfile --network host --build-arg HTTP_PROXY=http://127.0.0.1:7890 --build-arg HTTPS_PROXY=http://127.0.0.1:7890 --build-arg NO_PROXY=localhost,127.0.0.1 -t mplib-manylinux2014 .
```

When running with proxy, prepend the proxy settings to the dockerfile as well.
```dockerfile
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG NO_PROXY

ENV http_proxy=$HTTP_PROXY
ENV https_proxy=$HTTPS_PROXY
ENV no_proxy=$NO_PROXY
```

## launch docker
```bash
sudo docker run --rm -it --network host \          
  -v "$(pwd)":/workspace \
  -w /workspace \
  mplib-manylinux2014 \
  /bin/bash
```

## build wheel
Before buiding wheel, we should have the python version we want to build wheel for installed in the docker container, which is specified in the last few lines of the dockerfile. For distribution to other people or other machines, use `auditwheel` to wrap all the dependencies into the wheel file.
```bash
pip wheel . -w dist -v
# the wheel file will be in the ./dist folder

# replace cp3xx with the python version we have
auditwheel repair dist/mplib-0.2.0-cp312-cp312-linux_x86_64.whl -w repaired_wheels 
# the repaired wheel file will be in the ./repaired_wheels folder
```
Now we can just `pip install` the repaired wheel file on any (linux) machine with the same python version.
