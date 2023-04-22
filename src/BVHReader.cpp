#include "BVHReader.h"
#include "Skeleton.h"
#include <iostream>
#include <stack>
#include "Link.h"
#include <assert.h>
#include "AnimRec.h"

#define KP_XPOSITION 1;
#define KP_YPOSITION 2;
#define KP_ZPOSITION 3;

#define KP_XROTATION 4;
#define KP_YROTATION 5;
#define KP_ZROTATION 6;


void trim(const char* in, char* out)
{
	strcpy(out, in);
	int len = strlen(out);
	for (int x = len - 1; x >= 0; x--)
	{
		if (out[x] == '\n' || out[x] == '\r')
			out[x] = '\0';
	}
}

//This code is based on a bvh reader from the DANCE framework, likely written by Ari Shapiro
void BVHReader::BuildSkelFromHeader(std::ifstream& file, Skeleton* newSkel, AnimRec* pAnimRec, bool inToM)
{

	// check to make sure we have properly opened the file
	if (!file.good())
	{
		std::cerr << "Could not open file in CKinSkelParseBVH::Parse\n";
		return;
	}
	char line[8192];
	int state = 0;
	char* str = NULL;
	std::stack<Link*> stack;
	Link* curLink = NULL;
	int numFrames = 0;
	int curFrame = -1;
	double frameTime = 0;
	int foundRoot = 0; // 0 = root not found, 1 = root found, 2 = next joint found
	int numRot = 0;
	int numTrans = 0;
	int totalDOFs = 0;

	while (!file.eof() && file.good())
	{
		file.getline(line, 8192, '\n');
		// remove any trailing \r
		if (line[strlen(line) - 1] == '\r')
			line[strlen(line) - 1] = '\0';
		if (strlen(line) == 0) // ignore blank lines
			continue;
		switch (state)
		{
		case 0:	// looking for 'HIERARCHY'
			str = strtok(line, " \t");
			if (strncmp(str, "HIERARCHY", strlen("HIERARCHY")) == 0)
				state = 1;
			else
			{
				std::cerr << "HIERARCHY not found...\n";
				file.close();
				return;
			}
			break;
		case 1:	// looking for 'ROOT'
			str = strtok(line, " \t");
			if (str != NULL && strncmp(str, "ROOT", strlen("ROOT")) == 0)
			{
				str = strtok(NULL, " \t");
				if (str != NULL)
				{
					curLink = new Link();
					curLink->SetName(str);

					//put link in tree
					newSkel->AddToSkeleton(curLink, (char *)"$ground");

					curLink->SetJointType("free");

					state = 2;
				}
				else
				{
					std::cerr << "ROOT name not found...\n";
					file.close();
					return;
				}
			}
			else
			{
				std::cerr << "ROOT not found...\n";
				file.close();
				return;
			}
			break;
		case 2: // looking for '{'
			str = strtok(line, " \t");
			if (str != NULL && strncmp(str, "{", 1) == 0)
			{
				stack.push(curLink);
				state = 3;
			}
			else
			{
				std::cerr << "{ not found...\n";
				file.close();
				return;
			}
			break;
		case 3: // looking for 'OFFSET'
			str = strtok(line, " \t");
			if (str != NULL && strncmp(str, "OFFSET", strlen("OFFSET")) == 0)
			{
				if (foundRoot == 0)
					foundRoot = 1;
				else if (foundRoot == 1)
					foundRoot = 2;
				double x = 0; double y = 0; double z = 0;
				str = strtok(NULL, " \t");
				x = atof(str);
				str = strtok(NULL, " \t");
				y = atof(str);
				str = strtok(NULL, " \t");
				z = atof(str);
				curLink->SetParTranslation(x, y, z);
				//cout << "Found offset of " << x << " " << y << " " << z << " " << endl;
				state = 4;
			}
			else
			{
				std::cerr << "OFFSET not found...\n";
				file.close();
				return;
			}
			break;
		case 4: // looking for 'CHANNELS'
			str = strtok(line, " \t");
			numRot = 0;
			numTrans = 0;
			if (str != NULL && strncmp(str, "CHANNELS", strlen("CHANNELS")) == 0)
			{
				str = strtok(NULL, " \t");
				int numChannels = atoi(str);

				// make sure that only the root has > 3 channels
				bool ignoreChannels = false;
				if (numChannels > 3 && foundRoot == 2)
				{
					std::cerr << "Too many channels (%d) found for non-root node at %s, reducing to %d..."<< numChannels<< curLink->GetName() << (numChannels - 3) << std::endl;
					ignoreChannels = true;
				}
				int channels[6] = { 0 };
				//std::cerr << "Found %d channels...\n", numChannels);
				for (int c = 0; c < numChannels; c++)
				{
					str = strtok(NULL, " \t");
					int axisNum = c;
					if (c > 2) axisNum -= 3;
					if (strncmp(str, "Xrotation", strlen("Xrotation")) == 0)
					{
						curLink->SetAxisOrder(axisNum, 0);
						channels[c] = KP_XROTATION;
						numRot++;
					}
					else if (strncmp(str, "Yrotation", strlen("Yrotation")) == 0)
					{
						curLink->SetAxisOrder(axisNum, 1);
						channels[c] = KP_YROTATION;
						numRot++;
					}
					else if (strncmp(str, "Zrotation", strlen("Zrotation")) == 0)
					{
						curLink->SetAxisOrder(axisNum, 2);
						channels[c] = KP_ZROTATION;
						numRot++;
					}
					else if (strncmp(str, "Xposition", strlen("Xposition")) == 0)
					{
						channels[c] = KP_XPOSITION;
						numTrans++;
					}
					else if (strncmp(str, "Yposition", strlen("Yposition")) == 0)
					{
						channels[c] = KP_YPOSITION;
						numTrans++;
					}
					else if (strncmp(str, "Zposition", strlen("Zposition")) == 0)
					{
						channels[c] = KP_ZPOSITION;
						numTrans++;
					}
					else
					{
						std::cerr << "Unknown channel: %s...\n" << str;
						file.close();
					}
				}
				if (ignoreChannels)
				{
					assert(false);//from ParseBVH, but doesn't seem right
					for (int i = 0; i < 3; i++)
						channels[i] = channels[i + 3];
					numChannels -= 3;
				}

				if (foundRoot == 2)
				{
					assert(numTrans == 0 && numRot > 0);
				}

				char jointType[100];
				if (numChannels == 6)
				{
					//represent this using the bvh native euler angle representation
					strcpy(jointType, "eulersix");

					//						strcpy(jointType, "free");
				}
				else if (numRot == 1)
				{
					strcpy(jointType, "pin");
				}
				else if (numRot == 2)
				{
					strcpy(jointType, "ujoint");
				}
				else if (numRot == 3)
				{
					strcpy(jointType, "gimbal");
					//NOTE: NOT CURRENTLY ALLOWING FOR BALL JOINTS
				}
				else
				{
					assert(false);
				}

				totalDOFs += numChannels;

				curLink->SetJointType(jointType);

				state = 5;
			}
			else
			{
				std::cerr << "CHANNELS not found...\n";
				file.close();
				return;
			}
			break;
		case 5: // looking for 'JOINT' or 'End Site' or '}' or 'MOTION'
			str = strtok(line, " \t");
			if (strncmp(str, "JOINT", strlen("JOINT")) == 0)
			{
				str = strtok(NULL, "");
				if (str != NULL)
				{
					char trimmedname[512];
					trim(str, trimmedname);
					curLink = new Link();
					curLink->SetName(trimmedname);

					//						CharJoint* joint = new CharJoint(trimmedname);
					//						CharJoint* top = stack.top();
					//						top->addChild(joint);
					//						joint->setParent(top);
					//						cur = joint;

					Link* top = stack.top();
					newSkel->AddToSkeleton(curLink, top->GetName());
					//stack.push(trimmedname);




					//cout << "Found joint " << str << endl;
					state = 2;
				}
				else
				{
					std::cerr << "ROOT name not found...\n";
					file.close();
					return;
				}
			}
			else if (strncmp(str, "End", strlen("End")) == 0)
			{
				str = strtok(NULL, " \t");
				if (strncmp(str, "Site", strlen("Site")) == 0)
				{
					state = 6;
				}
				else
				{
					std::cerr << "End site not found...\n";
					file.close();
					return;
				}
			}
			else if (strncmp(str, "}", 1) == 0)
			{
				str = strtok(line, " \t");
				if (str != NULL && strncmp(str, "}", 1) == 0)
				{
					stack.pop();
					state = 5;
				}
				else
				{
					std::cerr << "} not found...\n";
					file.close();
					return;
				}
			}
			else if (strncmp(str, "MOTION", strlen("MOTION")) == 0)
			{
				state = 9;
			}
			else
			{
				std::cerr << "JOINT or End Site not found...\n";
				file.close();
				return;
			}
			break;
		case 6: // looking for 'OFFSET' within end effector
			str = strtok(line, " \t");
			if (str != NULL && strncmp(str, "{", 1) == 0)
			{
				state = 7;
			}
			else
			{
				std::cerr << "{ not found for end effector...\n";
				std::cerr << "{ not found for end effector..." << std::endl;
				file.close();
				return;
			}
			break;
		case 7:
			str = strtok(line, " \t");
			if (str != NULL && strncmp(str, "OFFSET", strlen("OFFSET")) == 0)
			{
				//This is for the end effector
				double x = 0; double y = 0; double z = 0;
				str = strtok(NULL, " \t");
				x = atof(str);
				str = strtok(NULL, " \t");
				y = atof(str);
				str = strtok(NULL, " \t");
				z = atof(str);
				
				//Creating new Link for end site
				curLink = new Link();
				Link* top = stack.top();

				// Adding end site to skeleton and linking to parent
				newSkel->AddToSkeleton(curLink, top->GetName());

				// Setting end site joint type to weld as it has 0 dof
				curLink->SetJointType("weld");

				// Setting m_parTrans for end site
				curLink->SetParTranslation(x, y, z);

				//TO DO: add this into the representation so that the last link can be drawn
				//					cur->setEndEffectorOffset(x, y, z);
				//					cur->setEndEffector(true);
									//std::cerr << "Found end effector at %s", cur->getName());
									//cout << "Found end effector offset of " << x << " " << y << " " << z << " " << endl;
				state = 8;
			}
			else
			{
				std::cerr << "End effector OFFSET not found...\n";
				file.close();
				return;
			}
			break;
		case 8: // looking for '}' to finish the  end effector
			str = strtok(line, " \t");
			if (str != NULL && strncmp(str, "}", 1) == 0)
			{
				state = 5;
			}
			else
			{
				std::cerr << "} not found for end effector...\n";
				file.close();
				return;
			}
			break;
		case 9: // found 'MOTION', looking for 'Frames'
			str = strtok(line, ":");
			if (str != NULL && strncmp(str, "Frames", strlen("Frames")) == 0)
			{
				str = strtok(NULL, " \t");
				numFrames = atoi(str);
				std::cout << "Found " << numFrames << " frames of animation...\n"<< std::endl;
				state = 10;
			}
			else
			{
				std::cerr << "Frames: not found...\n";;
				file.close();
				return;
			}
			break;
		case 10: // found 'Frames', looking for 'Frame time:'
			str = strtok(line, ":");
			if (str != NULL && strncmp(str, "Frame Time", strlen("Frame Time")) == 0)
			{
				str = strtok(NULL, " \t");
				frameTime = atof(str);
				std::cout << "Frame time is: "<< frameTime << std::endl;
				pAnimRec->SetFrameTime(frameTime);
				//curFrame = 0;
				state = 11;
			}
			else
			{
				std::cerr << "Frame Time: not found...\n";
				file.close();
				return;
			}
			break;
		case 11: // parsing 
			curFrame++;

			pAnimRec->SetNumDOFs(totalDOFs);

			{
				if (curFrame < numFrames)
				{
					pAnimRec->StoreLine(line, inToM);
					state = 11;
				}
				else
				{
					state = 12;
				}
			}

			break;
		case 12:
			state = 50;
			break;

		case 50:
			std::cout << "Finished parsing motion with %d frames... " << numFrames << std::endl;
			file.close();



			return;
		default:
			std::cerr << "State " << state << " not expected..." << std::endl;
			file.close();
			return;
		}
	}

}