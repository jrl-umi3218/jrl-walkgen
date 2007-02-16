#include "fileReader.h"

namespace PatternGeneratorJRL
{

// Search for string
char look_for(FILE *fichier, const char *str)
{
  int i = 0;
  char c;
  bool Cont=false;

  do
    {
      fscanf(fichier, "%c", &c);

      if (Cont && c=='\n')
	Cont=false;
      else if (c=='#')
	Cont=true;
      
      if (!Cont)
	if (c == str[i] && i < (int)(strlen(str)))
	  i++;
	else
	  if (i == (int)strlen(str))
	    return 1;
	  else
	    i = 0;
    }
  while (!feof(fichier));
    
  return 0;
}

bool immediatlyAppears(FILE* fichier, const char *str) 
{
  bool b = true;
  char c;
  fpos_t afpos_t;

  fgetpos(fichier,&afpos_t);
  for (int i=0; i<(int)(strlen(str)) && b; i++) {
    if(!feof(fichier))
      {
	fscanf(fichier, "%c", &c);
	b = (c==str[i]);
      }
    else
      {
	fsetpos(fichier,&afpos_t);
	return 0;
      }
  }
  if (b==false)
    fsetpos(fichier,&afpos_t);

  if ((!strcmp(str,"ointId")) && (b==false))
    {
      char Buffer[124];
      fread(Buffer,strlen(str),1,fichier);
      cout << "Refused by immediatlyAppears" << Buffer << " ";
      fsetpos(fichier,&afpos_t);
	    
    }

  return b;
}

int nextKeyWord(FILE* fich)
{
  char c;
	
  while (!feof(fich))
    {
      fscanf(fich, "%c", &c);
      switch (c) {
      case '[' :
	return CROCHET_OUVRANT;
	break;
      case ']' :
	return CROCHET_FERMANT;
	break;
      case 'c' :
	if (immediatlyAppears(fich, "hildren")) {
	  return CHILDREN;
	}
	break;
      case 'J' :
	if (immediatlyAppears(fich, "oint {")) {
	  return JOINT;
	}
	break;
      case 'D' :
	if (immediatlyAppears(fich, "EF")) {
	  return DEF;
	}
	break;

      }
    }
  return -2;
}

int nextJointKeyWord(FILE* fichier)
{
  char c;
  do {
    fscanf(fichier, "%c", &c);
    switch (c) {
    case 'j' :
      if (immediatlyAppears(fichier, "ointAxis \"")) {
	fscanf(fichier, "%c", &c);
	switch (c) {
	case 'X' :
	  return AXE_X;
	case 'Y' :
	  return AXE_Y;
	case 'Z' :
	  return AXE_Z;
	}
      }
      else if (immediatlyAppears(fichier, "ointId ")) {
	return JOINT_ID;
      }
      break;
    case 't' :
      if (immediatlyAppears(fichier, "ranslation")) {
	return JOINT_TRANSLATION;
      }
      break;
    case 'r' :
      if (immediatlyAppears(fichier, "otation")) {
	return JOINT_ROTATION;
      }
      break;
      break;
    }

  } while (!feof(fichier));
  return 0;
}

int typeOfJoint(FILE* fichier)
{
  look_for(fichier,"jointType");
  char c;
  fscanf(fichier, "%c", &c);
  fscanf(fichier, "%c", &c);
  switch (c) {
  case 'f' :
    if (immediatlyAppears(fichier,"ree")) {
      return -1;
    }
    break;
  case 'r' :
    if (immediatlyAppears(fichier,"otate")) {
      return 1;
    }
  }
  return 0;
}
};
