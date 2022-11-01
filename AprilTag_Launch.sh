#!/bin/sh
skip=49

tab='	'
nl='
'
IFS=" $tab$nl"

umask=`umask`
umask 77

gztmpdir=
trap 'res=$?
  test -n "$gztmpdir" && rm -fr "$gztmpdir"
  (exit $res); exit $res
' 0 1 2 3 5 10 13 15

case $TMPDIR in
  / | /*/) ;;
  /*) TMPDIR=$TMPDIR/;;
  *) TMPDIR=/tmp/;;
esac
if type mktemp >/dev/null 2>&1; then
  gztmpdir=`mktemp -d "${TMPDIR}gztmpXXXXXXXXX"`
else
  gztmpdir=${TMPDIR}gztmp$$; mkdir $gztmpdir
fi || { (exit 127); exit 127; }

gztmp=$gztmpdir/$0
case $0 in
-* | */*'
') mkdir -p "$gztmp" && rm -r "$gztmp";;
*/*) gztmp=$gztmpdir/`basename "$0"`;;
esac || { (exit 127); exit 127; }

case `printf 'X\n' | tail -n +1 2>/dev/null` in
X) tail_n=-n;;
*) tail_n=;;
esac
if tail $tail_n +$skip <"$0" | gzip -cd > "$gztmp"; then
  umask $umask
  chmod 700 "$gztmp"
  (sleep 5; rm -fr "$gztmpdir") 2>/dev/null &
  "$gztmp" ${1+"$@"}; res=$?
else
  printf >&2 '%s\n' "Cannot decompress $0"
  (exit 127); res=127
fi; exit $res
�qacAprilTag_Launch.sh ��1O�0�w���Aݜ��:!*!120�!9�5�p�U�įǍ	�hR�%���߻���ę��V�� �5J���d��Qq�{WZ��BcS�^�Ӻ�NEZ��W�in�)jMk��N
�\�Gj�D.U�5�f����Ո;H,cj��oWGo�UH��g���,�x�(-TJ25�æV$u7㡹�Z+�]��^��	����L�
D�1���A�-��$��mA����cj���O̻��qm�	##:Y����S_:���(�8�@~�|�k|��m����SW��!����/�5���-W�Q��Nz��o��b(��;�Hp�k\;�~3Ď�V1�E�@��Z�o�T�ˡ  