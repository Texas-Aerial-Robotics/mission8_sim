# Generates a QR Code from user input. 
	echo "Type in the 4 digits of the QRCode"
	read i

	height=145              # height of cropped image
	width=145               # width of cropped image

	
	# Generate top left image
	qr "$i" | convert - -crop "$height"x"$width"+0+0 qr_topLeft.png

	# Generate top right image
	qr "$i" | convert - -crop "$height"x"$width"+"$width"+0 qr_topRight.png

	# Generate bottom left image
	qr "$i" | convert - -crop "$height"x"$width"+0+"$height" qr_bottomLeft.png

	# Generate bottom right image
	qr "$i" | convert - -crop "$height"x"$width"+"$width"+"$height" qr_bottomRight.png
	
