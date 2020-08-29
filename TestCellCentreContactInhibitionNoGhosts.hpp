/*

Copyright (c) 2005-2018, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef TESTRUNNIGCONTACTINHIBITIONSIMULATIONSTUTORIAL_HPP_
#define TESTRUNNIGCONTACTINHIBITIONSIMULATIONSTUTORIAL_HPP_

#ifndef TESTCREATINGANDUSINGANEWCELLPOPULATIONBOUNDARYCONDITIONTUTORIAL_HPP_
#define TESTCREATINGANDUSINGANEWCELLPOPULATIONBOUNDARYCONDITIONTUTORIAL_HPP_

#include <cxxtest/TestSuite.h>
#include "CheckpointArchiveTypes.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "SmartPointers.hpp"
#include "Exception.hpp"
#include "ContactInhibitionCellCycleModel.hpp"
#include "VolumeTrackingModifier.hpp"
#include "OffLatticeSimulation.hpp"
#include "MeshBasedCellPopulation.hpp"
#include "CellMutationStatesCountWriter.hpp"
#include "WildTypeCellMutationState.hpp"
#include "StemCellProliferativeType.hpp"
#include "TransitCellProliferativeType.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "NagaiHondaForce.hpp"
#include "SimpleTargetAreaModifier.hpp"
#include "SimulationTime.hpp"
#include "CellLabel.hpp"
#include "MutableMesh.hpp"
#include "MutableVertexMesh.hpp"
#include "PlaneBoundaryCondition.hpp"
#include "FakePetscSetup.hpp"

#include "CellPopulationCounterWriter.hpp"
#include "CellPopulationAreaWriter.hpp"

#include "CellAncestorWriter.hpp"
#include "CellIdWriter.hpp"

#include "WildTypeCellMutationState.hpp"
#include "StemCellProliferativeType.hpp"
#include "TransitCellProliferativeType.hpp"
#include "CellProliferativeTypesCountWriter.hpp"
#include "AbstractCellPopulationBoundaryCondition.hpp"
#include "CylindricalHoneycombMeshGenerator.hpp"

#include "MeshBasedCellPopulationWithGhostNodes.hpp"

#include <math.h>
#include <time.h>

class MyBoundaryCondition : public AbstractCellPopulationBoundaryCondition<2>
{
private:

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCellPopulationBoundaryCondition<2> >(*this);
    }

public:
    MyBoundaryCondition(AbstractCellPopulation<2>* pCellPopulation)
        : AbstractCellPopulationBoundaryCondition<2>(pCellPopulation)
    {
    }
    // Restric cells to a specified radius
    void ImposeBoundaryCondition(const std::map<Node<2>*, c_vector<double, 2> >& rOldLocations)
    {
        

        // specify the domains parameters. For square structues, the radius is actually the half length
        double domainRadius = 50.0;
        double magPoint;
        // Iterate through cell population and enforce each cell be inside the domain
        for (AbstractCellPopulation<2>::Iterator cell_iter = this->mpCellPopulation->Begin();
             cell_iter != this->mpCellPopulation->End();
             ++cell_iter)
        {
            unsigned node_index = this->mpCellPopulation->GetLocationIndexUsingCell(*cell_iter);
            Node<2>* p_node = this->mpCellPopulation->GetNode(node_index);
            double y_coordinate = p_node->rGetLocation()[1];
            double x_coordinate = p_node->rGetLocation()[0];
            

            //Box 'domainRadius' x 'domainRadius'
            if (y_coordinate > domainRadius)
            {
                p_node->rGetModifiableLocation()[1] = domainRadius;
            }
            else if (y_coordinate < -1.0*domainRadius)
            {
                p_node->rGetModifiableLocation()[1] = -1.0*domainRadius;
            }
            if (x_coordinate > domainRadius)
            {
                p_node->rGetModifiableLocation()[0] = domainRadius;
            }
            else if (x_coordinate < -1.0*domainRadius)
            {
                p_node->rGetModifiableLocation()[0] = -1.0*domainRadius;
            }

            //Circle radius domainRadius
            /*
            magPoint = y_coordinate*y_coordinate + x_coordinate*x_coordinate;
            if (magPoint > domainRadius*domainRadius)
            {
                p_node->rGetModifiableLocation()[0] = domainRadius*(x_coordinate)/(sqrt(magPoint));
                p_node->rGetModifiableLocation()[1] = domainRadius*(y_coordinate)/(sqrt(magPoint));
            }
            */

            //Toroidal 'domainRadius' x 'domainRadius'
            /*
            if (y_coordinate > domainRadius)
            {
                p_node->rGetModifiableLocation()[1] = y_coordinate - domainRadius;
            }
            else if (y_coordinate < 0.0)
            {
                p_node->rGetModifiableLocation()[1] = y_coordinate + domainRadius;
            }
            if (x_coordinate > domainRadius)
            {
                p_node->rGetModifiableLocation()[0] = x_coordinate - domainRadius;
            }
            else if (x_coordinate < 0.0)
            {
                p_node->rGetModifiableLocation()[0] = x_coordinate + domainRadius;
            }
            */
            
        }
    }

    bool VerifyBoundaryCondition()
    {
        bool condition_satisfied = true;
        double domainRadius = 50.0;
        double magPoint;
        for (AbstractCellPopulation<2>::Iterator cell_iter = this->mpCellPopulation->Begin();
             cell_iter != this->mpCellPopulation->End();
             ++cell_iter)
        {
            c_vector<double, 2> cell_location = this->mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
            double y_coordinate = cell_location(1);
            double x_coordinate = cell_location(0);

           //Circle radius  domainRadius
           /*
            magPoint = y_coordinate*y_coordinate + x_coordinate*x_coordinate;
            if (magPoint > 1.0001*domainRadius*domainRadius)
            {
                condition_satisfied = false;
                break;
            }
            */

            //Toroidal or box domain 'domainRadius' x 'domainRadius'
            if ((y_coordinate < -1.0*domainRadius) || (y_coordinate > domainRadius))
            {
                condition_satisfied = false;
                break;
            }
            if ((x_coordinate < -1.0*domainRadius) || (x_coordinate > domainRadius))
            {
                condition_satisfied = false;
                break;
            }
        }
        return condition_satisfied;
    }

    void OutputCellPopulationBoundaryConditionParameters(out_stream& rParamsFile)
    {
        AbstractCellPopulationBoundaryCondition<2>::OutputCellPopulationBoundaryConditionParameters(rParamsFile);
    }
};

#include "SerializationExportWrapper.hpp"
CHASTE_CLASS_EXPORT(MyBoundaryCondition)
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(MyBoundaryCondition)

namespace boost
{
    namespace serialization
    {
        template<class Archive>
        inline void save_construct_data(
            Archive & ar, const MyBoundaryCondition * t, const unsigned int file_version)
        {
            const AbstractCellPopulation<2>* const p_cell_population = t->GetCellPopulation();
            ar << p_cell_population;
        }

        template<class Archive>
        inline void load_construct_data(
            Archive & ar, MyBoundaryCondition * t, const unsigned int file_version)
        {
            AbstractCellPopulation<2>* p_cell_population;
            ar >> p_cell_population;

            ::new(t)MyBoundaryCondition(p_cell_population);
        }
    }
}

class TestRunningContactInhibitionSimulationsTutorial : public AbstractCellBasedTestSuite
{
public:
    void TestContactInhibitionInBox()
    {
        srand(time(NULL));
        // 0 -> no ghost nodes
        HoneycombMeshGenerator generator(4, 4, 0);
        MutableMesh<2,2>* p_mesh = generator.GetMesh();

        std::vector<CellPtr> cells;

        MAKE_PTR(WildTypeCellMutationState, p_state);
        MAKE_PTR(TransitCellProliferativeType, p_transit_type);

        //double x_inital[4] = {-7.5, 7.5, -7.5, 7.5};
        //double y_initial[4] = {-7.5, -7.5, 7.5, 7.5};

        for (unsigned i=0; i<p_mesh->GetNumNodes(); i++)
        {
            // Uniform random number [0,1]
            double unifRand = ((double) rand() / (RAND_MAX));
            // Uniform random number [0,12]
            double birth_time = -12.0*unifRand;
            
            ContactInhibitionCellCycleModel* p_cycle_model = new ContactInhibitionCellCycleModel();
            p_cycle_model->SetDimension(2);
            p_cycle_model->SetBirthTime(birth_time); //*(((double) rand() / (RAND_MAX)))
            p_cycle_model->SetQuiescentVolumeFraction(0.8);
            p_cycle_model->SetEquilibriumVolume(1.0);

            CellPtr p_cell(new Cell(p_state, p_cycle_model));
            p_cell->SetCellProliferativeType(p_transit_type);
            p_cell->InitialiseCellCycleModel();

            cells.push_back(p_cell);
        }

        MeshBasedCellPopulation<2> cell_population(*p_mesh, cells);
        
        MAKE_PTR_ARGS(MyBoundaryCondition, p_bc, (&cell_population));

        cell_population.AddCellPopulationCountWriter<CellMutationStatesCountWriter>();
        cell_population.AddPopulationWriter<CellPopulationCounterWriter>();
        cell_population.AddPopulationWriter<CellPopulationAreaWriter>();
        cell_population.AddCellPopulationCountWriter<CellProliferativeTypesCountWriter>();

        cell_population.SetCellAncestorsToLocationIndices();
        cell_population.AddCellWriter<CellAncestorWriter>();
        cell_population.AddCellWriter<CellIdWriter>();

        OffLatticeSimulation<2> simulator(cell_population);


        MAKE_PTR(VolumeTrackingModifier<2>, p_modifier);
        simulator.AddSimulationModifier(p_modifier);

        /* Next, we create a force law (springs) to be applied between cell centres and set up a
         * cut-off length beyond which cells stop interacting. We then pass this to the {{{VolumeTrackedOffLatticeSimulation}}}. */
        MAKE_PTR(GeneralisedLinearSpringForce<2>, p_force);
        p_force->SetCutOffLength(1.5);
        simulator.AddForce(p_force);

        simulator.AddCellPopulationBoundaryCondition(p_bc);

        simulator.SetDt(0.01);
        simulator.SetOutputDirectory("TestContactInhibitionNoGhosts");
        simulator.SetSamplingTimestepMultiple(10);
        simulator.SetEndTime(120.0);
        auto t1 = std::chrono::high_resolution_clock::now();
        simulator.Solve();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = pow(10.0,-6)*(std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count());

        std::cout << "\nTime taken = " << duration << " seconds\n\n";
    }
};

#endif /*TESTRUNNIGCONTACTINHIBITIONSIMULATIONSTUTORIAL_HPP_*/
#endif /*TESTCREATINGANDUSINGANEWCELLPOPULATIONBOUNDARYCONDITIONTUTORIAL_HPP_*/


/* To visualize the results, open a new terminal, {{{cd}}} to the Chaste directory,
* then {{{cd}}} to {{{anim}}}. Then do: {{{java Visualize2dCentreCells /tmp/$USER/testoutput/TestContactInhibitionNoGhosts/results_from_time_0}}}.
* We may have to do: {{{javac Visualize2dCentreCells.java}}} beforehand to create the
* java executable.
*/

