import qrcode
from qrcode.constants import ERROR_CORRECT_H
qr = qrcode.QRCode(
   version=1,
   error_correction=ERROR_CORRECT_H,
   box_size=10,
   border=0
)
# Uncomment one of these
qr.add_data("BIN_FLAG_SW")
#qr.add_data("BIN_FLAG_SE")
#qr.add_data("BIN_FLAG_NW")
#qr.add_data("BIN_FLAG_NE")
qr.make(fit=True)
img = qr.make_image()
img.save("QR.png")