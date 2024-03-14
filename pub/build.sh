#!/bin/bash
latex root.tex &&
bibtex root &&
latex root.tex &&
latex root.tex;
dvips -Ppdf -G0 root &&
ps2pdf \
    -dDownsampleColorImages=true \
    -dColorImageResolution=300 \
    -dGrayImageResolution=300 \
    -dMonoImageResolution=300 \
    root.ps root.pdf