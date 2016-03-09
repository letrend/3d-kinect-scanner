#include "ncurses_3dscanner.hpp"

int main(int argc, char *argv[]) {
	NCurses_3dscanner ncurse;
	char cmd;
	noecho();
	do{
		timeout(10);
		cmd = mvgetch(4,0);
		switch (cmd){
			case '0':
				ncurse.initialize();
				break;
			case '1':
				ncurse.scan();
				break;
            case '2':
                ncurse.saveMesh();
                break;
		}
	}while( cmd != '9');
	return 0;
//	if(scanner.intializeGridPosition()){
//		scanner.scan();
//		scanner.extractMesh();
//		scanner.saveMesh();
//		return 0;
//	}
//	std::cout << "could not initialize grid location" << std::endl;
//	return 1;
}
