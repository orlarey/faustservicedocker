# The <mdoc> blocks of a Faust program are copied as is into the LaTeX file
# that faust2mathdoc compiles. LaTeX reads any file by default (openin_any=a),
# so a program could include any file of the server in its documentation.
# This script makes faust2mathdoc compile with openin_any=p, which only reads
# the working directory and the TeX trees. The diagrams, referenced as ../svg/,
# are copied next to the LaTeX file during the compilation.
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "/usr/local/bin/faust2mathdoc"
text = open(path).read()
pattern = re.compile(
    r"^(\s*)cd tex && (xelatex|pdflatex) \$\{FILENAME%\.dsp\}\.tex && \2 \$\{FILENAME%\.dsp\}\.tex && cd \.\. &&$",
    re.M,
)
matches = pattern.findall(text)
if len(matches) != 1:
    sys.exit("restrict-mathdoc: expected one LaTeX line in %s, found %d" % (path, len(matches)))


def replace(m):
    indent, engine = m.group(1), m.group(2)
    return (
        indent + 'cd tex && cp -r ../svg svg && sed -i "s|\\.\\./svg/|svg/|g" ${FILENAME%.dsp}.tex &&\n'
        + indent + "openin_any=p " + engine + " ${FILENAME%.dsp}.tex && openin_any=p " + engine + " ${FILENAME%.dsp}.tex &&\n"
        + indent + "rm -rf svg && cd .. &&"
    )


open(path, "w").write(pattern.sub(replace, text))
print("restrict-mathdoc: %s now compiles with openin_any=p (%s)" % (path, matches[0][1]))
