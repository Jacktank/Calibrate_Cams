all:
	g++ main.cpp -g -o M `pkg-config --cflags --libs opencv320`
