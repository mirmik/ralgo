FROM igris-20.04
FROM nos-20.04
FROM crow-20.04
FROM netricks-20.04-env

COPY --from=0 /usr/local/include/igris /usr/local/include/igris
COPY --from=0 /usr/lib/libigris.so /usr/lib/libigris.so
COPY --from=1 /usr/local/include/nos /usr/local/include/nos
COPY --from=1 /usr/lib/libnos.so /usr/lib/libnos.so
COPY --from=2 /usr/local/include/crow /usr/local/include/crow
COPY --from=2 /usr/lib/libcrow.so /usr/lib/libcrow.so

ADD . /root/ralgo

WORKDIR /root/ralgo
RUN ./make.py
RUN sudo ./make.py install

WORKDIR /root/ralgo/tests
RUN ./make.py
RUN ./runtests