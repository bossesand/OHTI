
// https://stackoverflow.com/questions/17184813/how-to-encode-decode-ascii85-in-javascript

// https://en.wikipedia.org/wiki/Ascii85

// https://rfc.zeromq.org/spec:32/Z85/

function encode_ascii85(a) {
  var b, c, d, e, f, g, h, i, j, k;
  for (!/[^\x00-\xFF]/.test(a), 
    b = "\x00\x00\x00\x00".slice(a.length % 4 || 4), 
    a += b, 
    c = [], 
    d = 0, 
    e = a.length; e > d; d += 4) f = (a.charCodeAt(d) << 24) + (a.charCodeAt(d + 1) << 16) + (a.charCodeAt(d + 2) << 8) + a.charCodeAt(d + 3), 
    0 !== f ? (k = f % 85, f = (f - k) / 85, j = f % 85, f = (f - j) / 85, i = f % 85, 
    f = (f - i) / 85, h = f % 85, f = (f - h) / 85, 
    g = f % 85, c.push(g + 33, h + 33, i + 33, j + 33, k + 33)) :c.push(122);
  return function(a, b) {
    for (var c = b; c > 0; c--) a.pop();
  }(c, b.length), "<~" + String.fromCharCode.apply(String, c) + "~>";
}

function decode_ascii85(a) {
  var c, d, e, f, g, h = String, 
    l = "length", 
    w = 255, 
    x = "charCodeAt", 
    y = "slice", 
    z = "replace";
  for ("<~" === a[y](0, 2) && "~>" === a[y](-2), a = a[y](2, -2)[z](/\s/g, "")[z]("z", "!!!!!"), 
  c = "uuuuu"[y](a[l] % 5 || 5), a += c, e = [], f = 0, g = a[l]; g > f; f += 5) d = 52200625 * (a[x](f) - 33) + 614125 * (a[x](f + 1) - 33) + 7225 * (a[x](f + 2) - 33) + 85 * (a[x](f + 3) - 33) + (a[x](f + 4) - 33), 
  e.push(w & d >> 24, w & d >> 16, w & d >> 8, w & d);
  return function(a, b) {
    for (var c = b; c > 0; c--) a.pop();
  }(e, c[l]), h.fromCharCode.apply(h, e);
}

var myString='This is a test!';
var encoded=encode_ascii85(myString);
var decoded=decode_ascii85(encoded);
document.write(decoded);