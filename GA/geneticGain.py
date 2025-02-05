from deap import base, creator, tools, algorithms
from GAMAIN import tune
import random
import numpy as np
import pandas as pd

# Evaluation Function
def evaluate_H(individual):
    # Unpack Kp and Kd from the individual
    kp_h,kd_h = individual
    
    # Simulate the system with the given gains
    stateHistory = tune(kp_h=kp_h,kd_h=kd_h,tuneH=True)
    stateHistory = stateHistory.T

    z = stateHistory.iloc[0]
    h = stateHistory.iloc[1]
    theta = stateHistory.iloc[2]
    zDot = stateHistory.iloc[3]
    hDot = stateHistory.iloc[4]
    thetaDot = stateHistory.iloc[5]
    U_H = stateHistory.iloc[6]
    U_Z = stateHistory.iloc[7]
    time = stateHistory.iloc[-1]
    
    # Calculate fitness, e.g., based on state error or control effort
    # latitudinalError = (z.iloc[-1]-U_Z.iloc[-1])**2
    riseTimeError = (h.iloc[80]-U_H.iloc[-1])**2
    settlingError = (h.iloc[-1]-U_H.iloc[-1])**2

    overRotationError = 9e9 if theta.max() > np.pi or theta.min() < -np.pi else 0 
    fitness = riseTimeError+settlingError+overRotationError

    # Return a tuple with the fitness value
    return (fitness,)

def evaluate_Z(individual):
    # Unpack Kp and Kd from the individual
    kp_z,kd_z, kp_theta, kd_theta = individual
    
    # Simulate the system with the given gains
    stateHistory = tune(kp_z=kp_z,kd_z=kd_z, kp_theta=kp_theta,kd_theta=kd_theta, tuneZ=True)
    stateHistory = stateHistory.T

    z = stateHistory.iloc[0]
    h = stateHistory.iloc[1]
    theta = stateHistory.iloc[2]
    zDot = stateHistory.iloc[3]
    hDot = stateHistory.iloc[4]
    thetaDot = stateHistory.iloc[5]
    U_H = stateHistory.iloc[6]
    U_Z = stateHistory.iloc[7]
    time = stateHistory.iloc[-1]
    
    # Calculate fitness, e.g., based on state error or control effort
    horizontalTimeError = (z.iloc[80]-U_Z.iloc[-1])**2
    riseTimeError = (h.iloc[80]-U_H.iloc[-1])**2
    settlingError = (h.iloc[-1]-U_H.iloc[-1])**2

    overRotationError = 9e9 if theta.max() > np.pi or theta.min() < -np.pi else 0 
    fitness = riseTimeError+settlingError+overRotationError + horizontalTimeError

    # Return a tuple with the fitness value

    return (fitness,)


# Main function
def main(tuneH=False,tuneZ=False):

    # Define the types for individuals and fitness
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin)

    # Create the toolbox
    toolbox = base.Toolbox()

    # Define bounds for PD gains
    KP_MIN, KP_MAX = -10.0, 10.0  # Bounds for Kp
    KD_MIN, KD_MAX = -10.0, 10.0   # Bounds for Kd

    # Attribute generators
    if tuneH:
        toolbox.register("attr_kpH", random.uniform, KP_MIN, KP_MAX)
        toolbox.register("attr_kdH", random.uniform, KD_MIN, KD_MAX)
        # Structure of individuals and population
        toolbox.register("individual", tools.initCycle, 
                        creator.Individual, 
                        (toolbox.attr_kpH, toolbox.attr_kdH), n=1)
        toolbox.register("evaluate", evaluate_H)

    elif tuneZ:
        toolbox.register("attr_kpZ", random.uniform, KP_MIN, KP_MAX)
        toolbox.register("attr_kdZ", random.uniform, KD_MIN, KD_MAX)
        toolbox.register("attr_kpTheta", random.uniform, KP_MIN, KP_MAX)
        toolbox.register("attr_kdTheta", random.uniform, KD_MIN, KD_MAX)
    # Structure of individuals and population
        toolbox.register("individual", tools.initCycle, 
                        creator.Individual, 
                        (toolbox.attr_kpZ, toolbox.attr_kdZ,toolbox.attr_kpTheta, toolbox.attr_kdTheta), n=1)
        toolbox.register("evaluate", evaluate_Z)

    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    

    # Genetic operators
    toolbox.register("mate", tools.cxBlend, alpha=0.5)  # Blend crossover
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.1, indpb=0.2)  # Gaussian mutation
    toolbox.register("select", tools.selTournament, tournsize=3)  # Tournament selection
    # random.seed(42)
    population = toolbox.population(n=400)  # Create a population of 100 individuals
    currentBestFitness = 9e9
    currentBestIndividual = 'na'



    for gen in range(300):
        # Run the genetic algorithm
        algorithms.eaSimple(population,toolbox,cxpb=0.7,mutpb=0.4,ngen=1,verbose=False)

        # Find the best individual
        best_individual = tools.selBest(population,k=5)[0]
        if tuneH:best_fitness = evaluate_H(best_individual)[0]
        if tuneZ:best_fitness = evaluate_Z(best_individual)[0]
        
        if best_fitness < currentBestFitness:
            currentBestFitness = best_fitness
            currentBestIndividual = best_individual

        # Print generation info
        print(f"Generation {gen+1}: Best individual: {best_individual}\n Best fitness: {best_fitness:.1e}")
        print(f'Current Best Individual: {currentBestIndividual}\n Current Best Fitness: {currentBestFitness:.1e}\n')

if __name__ == "__main__":
    main(tuneZ=True)
    # X=evaluate_Z([0.004348755695726292, -0.20950792100260596, -5.211027023120496, -1.35736857233394])
