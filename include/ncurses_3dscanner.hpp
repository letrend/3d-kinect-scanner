#pragma once

#include "VolumeIntegration.cuh"
#include <ncurses.h>

enum COLORS{
	CYAN=1,
	RED,
	GREEN,
};
//! standard query messages
char welcomestring[] = "commandline tool for 3d kinect scanner";
char commandstring[] = "[0]initialize, [1]scan, [2]save mesh, [3]calibrate camera, [9]exit";
char scanningcubedimensionstring[] = "scanning cube dimension?";
char xdimstring[] = "xDim [default: 480]?  ";
char ydimstring[] = "yDim [default: 480]?  ";
char zdimstring[] = "zDim [default: 480]?  ";
char initializedstring[] = "initialized";
char voxelsizestring[] = "voxel size in meter [default: 0.01]?  ";
char initializinggridlocationstring[] = "initializing grid location";
char scanningstring[] = "scanning";
char runningstring[] = "running ";
char donestring[] = "done ";
char pressspacestring [] = "press space to start scanning";
char pressescapestring [] = "press escape to finish scanning";
char extractingmeshstring [] = "extracting mesh, this might take a while...";
char filenamestring [] = "filename [default: mesh.ply]?  ";
char savingstring [] = "saving...";
char upssomethingwentwrongstring [] = "ups...something went wrong";
char byebyestring[] = "BYE BYE!";

class NCurses_3dscanner{
public:
    NCurses_3dscanner(){
		//! start ncurses mode
		initscr();
		//! Start color functionality
		start_color();
		init_pair(CYAN, COLOR_CYAN, COLOR_BLACK);
		init_pair(RED, COLOR_RED, COLOR_BLACK);
		init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
		//! get the size of the terminal window
		getmaxyx(stdscr,rows,cols);

		print(0,0,cols,"-");
		printMessage(1,0,welcomestring);
		print(2,0,cols,"-");
		printMessage(3,0,commandstring);
	}
	~NCurses_3dscanner(){
        delete scanner;
		clearAll(0);
		printMessage(rows/2,cols/2-strlen(byebyestring)/2,byebyestring);
		refresh();
		usleep(1000000);
		endwin();
	}
	void initialize(){
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");
        print(6,0,cols," ");
        print(7,0,cols," ");
        print(8,0,cols," ");
		printMessage(4,0,scanningcubedimensionstring);
        // cube dimensions
        printMessage(5,0,xdimstring,CYAN);
        mvgetnstr(5,strlen(xdimstring),inputstring,30);
        uint xDim = atoi(inputstring);
        if(xDim==0) xDim = 480;

        printMessage(6,0,ydimstring,CYAN);
        mvgetnstr(6,strlen(ydimstring),inputstring,30);
        uint yDim = atoi(inputstring);
        if(yDim==0) yDim = 480;

        printMessage(7,0,zdimstring,CYAN);
        mvgetnstr(7,strlen(zdimstring),inputstring,30);
        uint zDim = atoi(inputstring);
        if(zDim==0) zDim = 480;

        printMessage(8,0,voxelsizestring,CYAN);
        mvgetnstr(8,strlen(voxelsizestring),inputstring,30);
        float voxelSize = atof(inputstring);
        if(voxelSize==0) voxelSize = 0.01;

        scanner = new VolumeIntegration(xDim,yDim,zDim,voxelSize);

        print(4,0,cols," ");
        print(5,0,cols," ");
        print(6,0,cols," ");
        print(7,0,cols," ");
        print(8,0,cols," ");

        printMessage(4,0,initializedstring,GREEN);
        usleep(1000000);
        print(4,0,cols," ");
		noecho();
	}
    void scan(){
        print(4,0,cols," ");
        printMessage(4,0,initializinggridlocationstring,CYAN);
        printMessage(5,0,pressspacestring,GREEN);
        if(scanner->intializeGridPosition()) {
            print(4,0,cols," ");
            print(5,0,cols," ");
            printMessage(4, 0, scanningstring, CYAN);
            printMessage(5, 0, pressescapestring,GREEN);
            scanner->scan();
            print(4,0,cols," ");
            print(5,0,cols," ");
            printMessage(4, 0, donestring, GREEN);
            printMessage(5, 0, extractingmeshstring, CYAN);
            scanner->extractMesh();
            print(4,0,cols," ");
            print(5,0,cols," ");
            printMessage(4, 0, donestring, GREEN);
            usleep(1000000);
            print(4,0,cols," ");
        }else{
            print(4,0,cols," ");
            printMessage(4,0,upssomethingwentwrongstring, RED);
        }
    }
    void saveMesh(){
        timeout(-1);
        echo();
        print(4,0,cols," ");
        printMessage(4,0,filenamestring, CYAN);
        mvgetnstr(4,strlen(filenamestring),inputstring,30);
        std::string name(inputstring);
        if(name.empty())
            name = "mesh.ply";
        print(4,0,cols," ");
        printMessage(4,0,savingstring, GREEN);
        if(scanner->saveMesh(name)){
            print(4,0,cols," ");
            printMessage(4,0,donestring, GREEN);
        }else{
            print(4,0,cols," ");
            printMessage(4,0,upssomethingwentwrongstring, RED);
        }
        usleep(1000000);
        print(4,0,cols," ");
    }
    void calibrate(){
        timeout(-1);
        echo();
        print(4,0,cols," ");
        scanner->calibrate();
    }

private:
    void processing(char* msg1, char* what, char* msg2){
        char cmd;
        uint a = strlen(msg1);
        uint b = strlen(what);
        uint c = strlen(msg2);
        print(5,0,cols," ");
        printMessage(5,0,msg1);
        printMessage(5,a+1, what);
        printMessage(5,a+1+b+1, msg2);
        mvchgat(5, 0,       a+1+b, A_BLINK, 2, NULL);
        mvchgat(5, a+1+b+1, a+1+b+1+c, A_BLINK, 1, NULL);
        timeout(10);
        do{
            cmd = mvgetch(5,a+1+b+1+c);
        }while(cmd != 'q');
        timeout(-1);
    }
    void processing(char* msg1, char* msg2){
        char cmd;
        uint a = strlen(msg1);
        uint c = strlen(msg2);

        print(5,0,cols," ");
        printMessage(5,0,msg1);
        printMessage(5,a+1, msg2);
        mvchgat(5, 0,       a, A_BLINK, 2, NULL);
        mvchgat(5, a+1, a+1+c, A_BLINK, 1, NULL);
        timeout(10);
        do{
            cmd = mvgetch(5,a+1+c);
        }while(cmd != 'q');
        timeout(-1);
    }
	void printMessage(uint row, uint col, char* msg){
		mvprintw(row,col,"%s", msg);
		refresh();
	}
	void printMessage(uint row, uint col, char* msg, uint color){
		mvprintw(row,col,"%s", msg);
		mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
		refresh();
	}
	void print(uint row, uint startcol, uint length, const char* s){
		for (uint i=startcol;i<startcol+length;i++){
			mvprintw(row,i,"%s",s);
		}
		refresh();
	}
	void clearAll(uint row){
		for(uint i=row;i<rows;i++){
			print(i,0,cols," ");
		}
		refresh();
	}
	VolumeIntegration *scanner;
	uint rows, cols;
	char inputstring[30];
};
