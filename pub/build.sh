#!/bin/bash
latex root.tex &&
bibtex root &&
latex root.tex &&
latex root.tex;
dvips -Ppdf -G0 root &&
ps2pdf \
    -dDownsampleColorImages=true \
    -dColorImageResolution=1200 \
    -dGrayImageResolution=1200 \
    -dMonoImageResolution=1200 \
    root.ps root.pdf