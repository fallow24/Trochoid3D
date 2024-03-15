#!/bin/bash
latex root.tex &&
bibtex root &&
latex root.tex &&
latex root.tex;
dvips -Ppdf -G0 root &&
ps2pdf \
    -dDownsampleColorImages=true \
    -dColorImageResolution=600 \
    -dGrayImageResolution=600 \
    -dMonoImageResolution=600 \
    root.ps root.pdf