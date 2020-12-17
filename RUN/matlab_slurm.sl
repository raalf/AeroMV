#!/bin/bash -l
#SBATCH --job-name=DJI
#SBATCH --account=def-brami # adjust this to match the accounting group you are using to submit jobs
#SBATCH --time=0-3:00         # adjust this to match the walltime of your job
#SBATCH --nodes=1      
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=12      # adjust this if you are using parallel commands
#SBATCH --mem=50000           # adjust this according to your the memory requirement per node you need
#SBATCH --mail-user=devin.barcelos@ryerson.ca # adjust this to match your email address
#SBATCH --mail-type=ALL

# Choose a version of MATLAB by loading a module:
module load matlab/2019a
# Remove -singleCompThread below if you are using parallel commands:
srun matlab -nodisplay -r "RUN_Matrice_Sensitivity"