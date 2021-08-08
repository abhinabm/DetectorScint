//
// ********************************************************************
// * License and Disclaimer                                           *
// *                                                                  *
// * The  Geant4 software  is  copyright of the Copyright Holders  of *
// * the Geant4 Collaboration.  It is provided  under  the terms  and *
// * conditions of the Geant4 Software License,  included in the file *
// * LICENSE and available at  http://cern.ch/geant4/license .  These *
// * include a list of copyright holders.                             *
// *                                                                  *
// * Neither the authors of this software system, nor their employing *
// * institutes,nor the agencies providing financial support for this *
// * work  make  any representation or  warranty, express or implied, *
// * regarding  this  software system or assume any liability for its *
// * use.  Please see the license in the file  LICENSE  and URL above *
// * for the full disclaimer and the limitation of liability.         *
// *                                                                  *
// * This  code  implementation is the result of  the  scientific and *
// * technical work of the GEANT4 collaboration.                      *
// * By using,  copying,  modifying or  distributing the software (or *
// * any work based  on the software)  you  agree  to acknowledge its *
// * use  in  resulting  scientific  publications,  and indicate your *
// * acceptance of all terms of the Geant4 Software license.          *
// ********************************************************************
//
//
/// \file DetectorConstruction.cc
/// \brief Adaptation of the B1DetectorConstruction class

#include "DetectorConstruction.hh"

#include "G4Material.hh"
#include "G4NistManager.hh"

#include "G4RunManager.hh"
#include "G4NistManager.hh"
#include "G4Box.hh"
#include "G4Tubs.hh"
#include "G4LogicalVolume.hh"

#include "G4PVPlacement.hh"
#include "G4PVReplica.hh"
#include "G4PVDivision.hh"

#include "G4PhysicalConstants.hh"
#include "G4SystemOfUnits.hh"

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

DetectorConstruction::DetectorConstruction()
: G4VUserDetectorConstruction(),
  fScoringVolume(0)
{ }

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

DetectorConstruction::~DetectorConstruction()
{ }

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::DefineMaterials()
{
	auto nistManager = G4NistManager::Instance();
	
	// Scintillator Material
	nistManager->FindOrBuildMaterial("G4_POLYSTYRENE");

  //   G4Material* scint_Mat = new G4Material ("plasticScint", 1.03*g/cm3, 1, kStateSolid);
 //   scint_Mat->AddMaterial(G4NistManager::Instance()->FindOrBuildMaterial("G4_POLYSTYRENE"), 1.0);


	// Air
	nistManager->FindOrBuildMaterial("G4_AIR");


	
 // ------------ Generate & Add Material Properties Table ------------
 /*
 G4double photonEnergy[] ={2.034 * eV, 2.068 * eV, 2.103 * eV, 2.139 * eV,
 2.177 * eV, 2.216 * eV, 2.256 * eV, 2.298 * eV,
 2.341 * eV, 2.386 * eV, 2.433 * eV, 2.481 * eV,
 2.532 * eV, 2.585 * eV, 2.640 * eV, 2.697 * eV,
 2.757 * eV, 2.820 * eV, 2.885 * eV, 2.954 * eV,
 3.026 * eV, 3.102 * eV, 3.181 * eV, 3.265 * eV,
 3.353 * eV, 3.446 * eV, 3.545 * eV, 3.649 * eV,
 3.760 * eV, 3.877 * eV, 4.002 * eV, 4.136 * eV};
 const G4int nEntries = sizeof (photonEnergy) / sizeof (G4double);
 //
 G4double refractiveIndex[] ={1.57 , 1.57 , 1.57 , 1.57 , 1.57 ,
 1.57 , 1.57 , 1.57 , 1.57 ,1.57 ,
 1.57 , 1.57 , 1.57 , 1.57 ,1.57 ,
 1.57 , 1.57 , 1.57 , 1.57  ,1.57 ,
 1.57 , 1.57 , 1.57 , 1.57 , 1.57 ,
 1.57 , 1.57 , 1.57 , 1.57 , 1.57 ,
 1.57 , 1.57 };
 G4MaterialPropertiesTable* myMPT1 = new G4MaterialPropertiesTable();
 myMPT1->AddProperty("RINDEX", photonEnergy, refractiveIndex, nEntries)->SetSpline(true);
 scint_Mat->SetMaterialPropertiesTable(myMPT1);
 
 */
}

G4VPhysicalVolume* DetectorConstruction::Construct()
{ 
	// Define materials
	DefineMaterials();

	auto air = G4Material::GetMaterial("G4_AIR");
	auto scintMat = G4Material::GetMaterial("G4_POLYSTYRENE");
	auto sensorMat = G4Material::GetMaterial("G4_POLYSTYRENE");
  
  	// Option to switch on/off checking of volumes overlaps
  	//
  	G4bool checkOverlaps = true; 
  //     
  // World
  //
  G4double world_sizeXY = 2.0*m;               
  G4double world_sizeZ  = 2.0*m;               

  
  auto solidWorld = new G4Box("World", 0.5*world_sizeXY, 0.5*world_sizeXY, 0.5*world_sizeZ);
  auto logicWorld = new G4LogicalVolume(solidWorld, air, "World");
  auto physWorld = new G4PVPlacement(0, G4ThreeVector(), logicWorld, "World", 0, false, 0, checkOverlaps);
               
    
 /* Abhinab's code for box*/    
  
  
    G4double ScintX = 5.*cm;
    G4double ScintY = 5.*cm;                      
    G4double ScintZ = 37.5*cm;                       // on each side adding to 75 cm total 
                         
    auto solidScint = new G4Box("Scint",ScintX, ScintY,ScintZ);
    auto logicScint = new G4LogicalVolume(solidScint, scintMat, "Scint");        
               
    G4double Scint_pX = 0.0*cm;
    G4double Scint_pY = 0.0*cm;
    G4double Scint_pZ = 0.0*cm;

    auto physTube = new G4PVPlacement(0, G4ThreeVector(Scint_pX, Scint_pY, Scint_pZ), logicScint, "Scint", logicWorld, false, 0, checkOverlaps);        
       
  
         
 /* Abhinab's code for scoring 'sensor' */        
         
    G4double sensor_pX = 5.0*cm;
    G4double sensor_pY = 5.0*cm;
    G4double sensor_pZ = 1.0*cm;
               
                            
    auto solidSensor= new G4Box("Sensor",sensor_pX, sensor_pY,sensor_pZ);
    auto logicSensor = new G4LogicalVolume(solidSensor, sensorMat, "Sensor");        
               
    G4double sensor_pos_x = 0.0*cm;
    G4double sensor_pos_y = 0.0*cm;
    G4double sensor_pos_z = 40.0*cm;

    auto physCube = new G4PVPlacement(0, G4ThreeVector(sensor_pos_x, sensor_pos_y, sensor_pos_z), logicSensor, "Sensor", logicWorld, false, 0, checkOverlaps);       
                 
  
  
  
  	// Set Mother as scoring volume
 	//
  	fScoringVolume = logicSensor;

  	//
  	//always return the physical World
  	//
  	return physWorld;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
