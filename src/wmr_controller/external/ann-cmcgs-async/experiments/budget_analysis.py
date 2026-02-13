#!/usr/bin/env python3
"""
Budget Analysis Script

This script runs experiments across multiple computational budgets and config files,
then generates CSV results and visualizations showing average reward vs computational budget.

Usage:
    python -m experiments.budget_analysis --configs configs/navigation_env_*.yaml --budgets 50,100,150,200,250 --planners MCGS,CMCGS --n_episodes 10

Features:
- Runs experiments for all combinations of configs, budgets, and planners
- Generates CSV file with results
- Creates visualization plots (budget vs reward)
- Supports parallel execution
- Saves results in structured format
"""

import os
import glob
import csv
import yaml
import numpy as np
import pandas as pd
import multiprocessing as mp
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Tuple, Any
import logging
import argparse

# Suppress matplotlib GUI to prevent figures from showing during optimization
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import seaborn as sns

from experiments.run_experiment import Runner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class BudgetAnalyzer:
    def __init__(self, 
                 config_files: List[str],
                 budgets: List[int],
                 planners: List[str],
                 n_episodes: int = 10,
                 max_steps_per_episode: int = None,
                 n_parallel: int = None,
                 output_dir: str = "logs/budget_analysis"):
        """
        Budget analyzer that runs experiments across budgets and configs.
        
        Args:
            config_files: List of config file paths
            budgets: List of computational budgets to test
            planners: List of planners to test ['MCGS', 'CMCGS']
            n_episodes: Number of episodes per experiment
            max_steps_per_episode: Max steps per episode (from config if None)
            n_parallel: Number of parallel processes (auto if None)
            output_dir: Directory to save results
        """
        self.config_files = config_files
        self.budgets = sorted(budgets)
        self.planners = planners
        self.n_episodes = n_episodes
        self.max_steps_per_episode = max_steps_per_episode
        self.n_parallel = n_parallel or min(mp.cpu_count(), 4)
        
        # Create output directory
        self.output_dir = Path(output_dir)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.output_dir / self.timestamp
        self.run_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"Created output directory: {self.run_dir}")
        
        # Results storage
        self.results = []
        
    def load_config(self, config_file: str) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    
    def run_single_experiment(self, config_file: str, planner: str, budget: int) -> Dict[str, Any]:
        """Run a single experiment and return results."""
        try:
            # Load config
            config = self.load_config(config_file)
            env_name = config['environment']['name']
            
            # Use max_steps from config if not specified
            max_steps = self.max_steps_per_episode or config['experiment']['max_steps']
            
            # Get base planner config
            base_planner_config = config['planners'][planner]['config'].copy()
            
            # For CMCGS, set simulation_budget in planner config
            if planner == 'CMCGS':
                base_planner_config['simulation_budget'] = budget
                comp_budget = config['experiment']['computational_budget_max']  # Use default
            else:
                # For MCGS, use budget as computational_budget_max
                comp_budget = budget
            
            # Create runner
            runner = Runner(
                env=env_name,
                planner=planner,
                computational_budget_max=comp_budget,
                time_budget_max=config['experiment']['time_budget_max'],
                env_config=config['environment'],
                planner_config=base_planner_config,
                render_mode=None,
                sleep_time=0.0,
                save_replay=False
            )
            
            # Run episodes
            rewards, steps = runner.run_multi_episode(
                num_episodes=self.n_episodes,
                max_steps_per_episode=max_steps
            )
            
            # Calculate statistics
            result = {
                'config_file': os.path.basename(config_file),
                'env_name': env_name,
                'planner': planner,
                'budget': budget,
                'n_episodes': self.n_episodes,
                'avg_reward': float(np.mean(rewards)),
                'std_reward': float(np.std(rewards)),
                'min_reward': float(np.min(rewards)),
                'max_reward': float(np.max(rewards)),
                'avg_steps': float(np.mean(steps)),
                'std_steps': float(np.std(steps)),
                'rewards': rewards,
                'steps': steps,
                'timestamp': datetime.now().isoformat()
            }
            
            logger.info(f"Completed: {env_name} | {planner} | Budget {budget} | Avg Reward: {result['avg_reward']:.3f}")
            return result
            
        except Exception as e:
            logger.error(f"Failed: {config_file} | {planner} | Budget {budget} | Error: {str(e)}")
            return {
                'config_file': os.path.basename(config_file),
                'env_name': 'ERROR',
                'planner': planner,
                'budget': budget,
                'n_episodes': 0,
                'avg_reward': -1000.0,
                'std_reward': 0.0,
                'min_reward': -1000.0,
                'max_reward': -1000.0,
                'avg_steps': 0.0,
                'std_steps': 0.0,
                'rewards': [],
                'steps': [],
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }
    
    def run_all_experiments(self) -> List[Dict[str, Any]]:
        """Run all experiment combinations."""
        # Generate all combinations
        experiments = []
        for config_file in self.config_files:
            for planner in self.planners:
                for budget in self.budgets:
                    experiments.append((config_file, planner, budget))
        
        logger.info(f"Running {len(experiments)} experiments with {self.n_parallel} parallel processes")
        logger.info(f"Configs: {len(self.config_files)}, Planners: {len(self.planners)}, Budgets: {len(self.budgets)}")
        
        # Run experiments in parallel
        if self.n_parallel > 1:
            with mp.Pool(processes=self.n_parallel) as pool:
                results = pool.starmap(self.run_single_experiment, experiments)
        else:
            results = [self.run_single_experiment(*exp) for exp in experiments]
        
        self.results = results
        return results
    
    def save_csv_results(self) -> str:
        """Save results to CSV file."""
        if not self.results:
            logger.warning("No results to save")
            return ""
        
        # Prepare data for CSV (excluding raw rewards/steps lists)
        csv_data = []
        for result in self.results:
            csv_row = {k: v for k, v in result.items() if k not in ['rewards', 'steps']}
            csv_data.append(csv_row)
        
        # Save to CSV
        csv_file = self.run_dir / f"budget_analysis_results.csv"
        df = pd.DataFrame(csv_data)
        df.to_csv(csv_file, index=False)
        
        logger.info(f"Saved CSV results to: {csv_file}")
        return str(csv_file)
    
    def save_detailed_json(self) -> str:
        """Save detailed results including raw data to JSON."""
        import json
        
        json_file = self.run_dir / f"budget_analysis_detailed.json"
        
        # Convert numpy types to Python types for JSON serialization
        json_results = []
        for result in self.results:
            json_result = {}
            for k, v in result.items():
                if isinstance(v, (np.ndarray, list)):
                    json_result[k] = [float(x) if isinstance(x, (np.number, np.floating, np.integer)) else x for x in v]
                elif isinstance(v, (np.number, np.floating, np.integer)):
                    json_result[k] = float(v)
                else:
                    json_result[k] = v
            json_results.append(json_result)
        
        with open(json_file, 'w') as f:
            json.dump({
                'metadata': {
                    'timestamp': self.timestamp,
                    'n_experiments': len(self.results),
                    'config_files': self.config_files,
                    'budgets': self.budgets,
                    'planners': self.planners,
                    'n_episodes': self.n_episodes,
                    'max_steps_per_episode': self.max_steps_per_episode
                },
                'results': json_results
            }, f, indent=2)
        
        logger.info(f"Saved detailed JSON to: {json_file}")
        return str(json_file)
    
    def create_visualizations(self) -> List[str]:
        """Create visualization plots."""
        if not self.results:
            logger.warning("No results to visualize")
            return []
        
        plot_files = []
        
        # Convert results to DataFrame for easier plotting
        df = pd.DataFrame([{k: v for k, v in r.items() if k not in ['rewards', 'steps']} for r in self.results])
        
        # Set style
        plt.style.use('seaborn-v0_8')
        sns.set_palette("husl")
        
        # 1. Budget vs Reward Boxplots (separate plot for each environment)
        environments = df['env_name'].unique()
        for env in environments:
            if env == 'ERROR':
                continue
                
            # Prepare data for boxplots - need to expand individual rewards
            env_results = [r for r in self.results if r['env_name'] == env]
            
            if not env_results:
                continue
            
            # Create data for boxplot
            boxplot_data = []
            labels = []
            colors = []
            palette = sns.color_palette("husl", len(self.planners))
            color_map = {planner: palette[i] for i, planner in enumerate(self.planners)}
            
            for planner in self.planners:
                for budget in sorted(self.budgets):
                    # Find matching result
                    matching_results = [r for r in env_results if r['planner'] == planner and r['budget'] == budget]
                    if matching_results and matching_results[0]['rewards']:
                        rewards = matching_results[0]['rewards']
                        boxplot_data.append(rewards)
                        labels.append(f"{planner}\nBudget {budget}")
                        colors.append(color_map[planner])
            
            if boxplot_data:
                plt.figure(figsize=(12, 8))
                
                # Create boxplot
                box_plot = plt.boxplot(boxplot_data, patch_artist=True,
                                     boxprops=dict(facecolor='lightblue', alpha=0.7),
                                     medianprops=dict(color='red', linewidth=2),
                                     whiskerprops=dict(color='black', linewidth=1.5),
                                     capprops=dict(color='black', linewidth=1.5))
                
                # Set custom labels
                plt.xticks(range(1, len(labels) + 1), labels)
                
                # Color boxes by planner
                for patch, color in zip(box_plot['boxes'], colors):
                    patch.set_facecolor(color)
                    patch.set_alpha(0.7)
                
                plt.xlabel('Planner and Budget', fontsize=12)
                plt.ylabel('Reward Distribution', fontsize=12)
                plt.title(f'Reward Distribution by Budget - {env}', fontsize=14, fontweight='bold')
                plt.xticks(rotation=45, ha='right')
                plt.grid(True, alpha=0.3, axis='y')
                plt.tight_layout()
                
                plot_file = self.run_dir / f"budget_reward_boxplot_{env.replace('-', '_')}.png"
                plt.savefig(plot_file, dpi=300, bbox_inches='tight')
                plt.close()
                plot_files.append(str(plot_file))
                logger.info(f"Saved boxplot: {plot_file}")
        
        # 2. Combined boxplot with all environments and planners
        if len(environments) > 1:
            # Prepare combined boxplot data
            combined_data = []
            combined_labels = []
            combined_colors = []
            
            palette = sns.color_palette("husl", len(environments) * len(self.planners))
            color_idx = 0
            
            for env in sorted(environments):
                if env == 'ERROR':
                    continue
                    
                env_results = [r for r in self.results if r['env_name'] == env]
                env_short = env.replace('NavigationEnv', '').replace('-v0', '')
                
                for planner in self.planners:
                    for budget in sorted(self.budgets):
                        matching_results = [r for r in env_results if r['planner'] == planner and r['budget'] == budget]
                        if matching_results and matching_results[0]['rewards']:
                            rewards = matching_results[0]['rewards']
                            combined_data.append(rewards)
                            combined_labels.append(f"{env_short}\n{planner}\nB{budget}")
                            combined_colors.append(palette[color_idx % len(palette)])
                    color_idx += 1
            
            if combined_data:
                plt.figure(figsize=(16, 10))
                
                box_plot = plt.boxplot(combined_data, patch_artist=True,
                                     boxprops=dict(facecolor='lightblue', alpha=0.7),
                                     medianprops=dict(color='red', linewidth=2),
                                     whiskerprops=dict(color='black', linewidth=1),
                                     capprops=dict(color='black', linewidth=1))
                
                # Set custom labels
                plt.xticks(range(1, len(combined_labels) + 1), combined_labels)
                
                # Color boxes
                for patch, color in zip(box_plot['boxes'], combined_colors):
                    patch.set_facecolor(color)
                    patch.set_alpha(0.7)
                
                plt.xlabel('Environment, Planner, and Budget', fontsize=12)
                plt.ylabel('Reward Distribution', fontsize=12)
                plt.title('Reward Distribution Across All Configurations', fontsize=14, fontweight='bold')
                plt.xticks(rotation=45, ha='right')
                plt.grid(True, alpha=0.3, axis='y')
                plt.tight_layout()
                
                combined_plot = self.run_dir / "budget_reward_boxplot_combined.png"
                plt.savefig(combined_plot, dpi=300, bbox_inches='tight')
                plt.close()
                plot_files.append(str(combined_plot))
                logger.info(f"Saved combined boxplot: {combined_plot}")
        
        # 3. Budget vs Average Reward Line Plot (for comparison)
        plt.figure(figsize=(12, 8))
        
        for env in environments:
            if env == 'ERROR':
                continue
            
            env_df = df[df['env_name'] == env]
            env_short = env.replace('NavigationEnv', '').replace('-v0', '')
            
            for planner in self.planners:
                planner_df = env_df[env_df['planner'] == planner]
                if not planner_df.empty:
                    label = f"{env_short} - {planner}"
                    plt.errorbar(planner_df['budget'], planner_df['avg_reward'], 
                               yerr=planner_df['std_reward'], 
                               marker='o', linewidth=2, markersize=6, 
                               label=label, capsize=3, alpha=0.8)
        
        plt.xlabel('Computational Budget', fontsize=12)
        plt.ylabel('Average Reward', fontsize=12)
        plt.title('Budget vs Average Reward - Line Plot Comparison', fontsize=14, fontweight='bold')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        
        line_plot = self.run_dir / "budget_vs_reward_lineplot.png"
        plt.savefig(line_plot, dpi=300, bbox_inches='tight')
        plt.close()
        plot_files.append(str(line_plot))
        logger.info(f"Saved line plot: {line_plot}")
        
        # 3. Heatmap of results
        if len(environments) > 1 and len(self.planners) > 1:
            for planner in self.planners:
                planner_df = df[df['planner'] == planner]
                if planner_df.empty:
                    continue
                
                # Create pivot table for heatmap
                pivot_df = planner_df.pivot(index='env_name', columns='budget', values='avg_reward')
                
                plt.figure(figsize=(12, 8))
                sns.heatmap(pivot_df, annot=True, fmt='.3f', cmap='viridis', 
                           cbar_kws={'label': 'Average Reward'})
                plt.title(f'Average Reward Heatmap - {planner}', fontsize=14, fontweight='bold')
                plt.xlabel('Computational Budget', fontsize=12)
                plt.ylabel('Environment', fontsize=12)
                plt.tight_layout()
                
                heatmap_file = self.run_dir / f"heatmap_{planner.lower()}.png"
                plt.savefig(heatmap_file, dpi=300, bbox_inches='tight')
                plt.close()
                plot_files.append(str(heatmap_file))
                logger.info(f"Saved heatmap: {heatmap_file}")
        
        # 4. Summary statistics plot
        plt.figure(figsize=(12, 8))
        
        # Box plot of rewards by planner
        df_clean = df[df['env_name'] != 'ERROR']
        if not df_clean.empty:
            sns.boxplot(data=df_clean, x='planner', y='avg_reward', hue='env_name')
            plt.title('Reward Distribution by Planner and Environment', fontsize=14, fontweight='bold')
            plt.ylabel('Average Reward', fontsize=12)
            plt.xlabel('Planner', fontsize=12)
            plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            plt.tight_layout()
            
            summary_plot = self.run_dir / "reward_distribution.png"
            plt.savefig(summary_plot, dpi=300, bbox_inches='tight')
            plt.close()
            plot_files.append(str(summary_plot))
            logger.info(f"Saved summary plot: {summary_plot}")
        
        return plot_files
    
    def generate_summary_report(self) -> str:
        """Generate a summary report."""
        if not self.results:
            return ""
        
        report_file = self.run_dir / "analysis_report.txt"
        
        with open(report_file, 'w') as f:
            f.write("BUDGET ANALYSIS REPORT\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total Experiments: {len(self.results)}\n")
            f.write(f"Config Files: {len(self.config_files)}\n")
            f.write(f"Planners: {', '.join(self.planners)}\n")
            f.write(f"Budgets: {', '.join(map(str, self.budgets))}\n")
            f.write(f"Episodes per Experiment: {self.n_episodes}\n\n")
            
            # Success rate
            successful = sum(1 for r in self.results if r['env_name'] != 'ERROR')
            f.write(f"Success Rate: {successful}/{len(self.results)} ({100*successful/len(self.results):.1f}%)\n\n")
            
            # Best results by environment
            df = pd.DataFrame([{k: v for k, v in r.items() if k not in ['rewards', 'steps']} for r in self.results])
            df_clean = df[df['env_name'] != 'ERROR']
            
            if not df_clean.empty:
                f.write("BEST RESULTS BY ENVIRONMENT:\n")
                f.write("-" * 30 + "\n")
                
                for env in df_clean['env_name'].unique():
                    env_df = df_clean[df_clean['env_name'] == env]
                    best_row = env_df.loc[env_df['avg_reward'].idxmax()]
                    
                    f.write(f"\n{env}:\n")
                    f.write(f"  Best: {best_row['planner']} with budget {best_row['budget']}\n")
                    f.write(f"  Reward: {best_row['avg_reward']:.4f} ± {best_row['std_reward']:.4f}\n")
                    f.write(f"  Steps: {best_row['avg_steps']:.1f} ± {best_row['std_steps']:.1f}\n")
                
                # Overall best
                overall_best = df_clean.loc[df_clean['avg_reward'].idxmax()]
                f.write(f"\nOVERALL BEST:\n")
                f.write(f"  {overall_best['env_name']} | {overall_best['planner']} | Budget {overall_best['budget']}\n")
                f.write(f"  Reward: {overall_best['avg_reward']:.4f} ± {overall_best['std_reward']:.4f}\n")
                
                # Budget efficiency analysis
                f.write(f"\nBUDGET EFFICIENCY ANALYSIS:\n")
                f.write("-" * 30 + "\n")
                
                for planner in self.planners:
                    planner_df = df_clean[df_clean['planner'] == planner]
                    if len(planner_df) > 1:
                        # Calculate efficiency as reward improvement per budget unit
                        budget_sorted = planner_df.sort_values('budget')
                        if len(budget_sorted) >= 2:
                            reward_diff = budget_sorted['avg_reward'].iloc[-1] - budget_sorted['avg_reward'].iloc[0]
                            budget_diff = budget_sorted['budget'].iloc[-1] - budget_sorted['budget'].iloc[0]
                            efficiency = reward_diff / budget_diff if budget_diff > 0 else 0
                            
                            f.write(f"\n{planner}:\n")
                            f.write(f"  Reward improvement: {reward_diff:.4f}\n")
                            f.write(f"  Budget range: {budget_sorted['budget'].iloc[0]} - {budget_sorted['budget'].iloc[-1]}\n")
                            f.write(f"  Efficiency: {efficiency:.6f} reward/budget\n")
        
        logger.info(f"Generated summary report: {report_file}")
        return str(report_file)
    
    def run_full_analysis(self) -> Dict[str, Any]:
        """Run complete budget analysis and return file paths."""
        logger.info("Starting budget analysis...")
        
        # Run experiments
        self.run_all_experiments()
        
        # Save results
        csv_file = self.save_csv_results()
        json_file = self.save_detailed_json()
        
        # Create visualizations
        plot_files = self.create_visualizations()
        
        # Generate report
        report_file = self.generate_summary_report()
        
        logger.info("Budget analysis complete!")
        logger.info(f"Results saved in: {self.run_dir}")
        
        return {
            'output_dir': str(self.run_dir),
            'csv_file': csv_file,
            'json_file': json_file,
            'plot_files': plot_files,
            'report_file': report_file
        }


def main():
    parser = argparse.ArgumentParser(description="Budget Analysis across multiple configs and budgets")
    parser.add_argument('--configs', required=True, help='Config files (glob pattern or comma-separated list)')
    parser.add_argument('--budgets', required=True, help='Comma-separated list of budgets (e.g., 50,100,150,200)')
    parser.add_argument('--planners', default='MCGS,CMCGS', help='Comma-separated list of planners')
    parser.add_argument('--n_episodes', type=int, default=10, help='Number of episodes per experiment')
    parser.add_argument('--max_steps', type=int, help='Max steps per episode (use config default if not specified)')
    parser.add_argument('--n_parallel', type=int, help='Number of parallel processes (auto-detect if not specified)')
    parser.add_argument('--output_dir', default='logs/budget_analysis', help='Output directory')
    parser.add_argument('--verbosity', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.verbosity))
    
    # Parse config files
    if '*' in args.configs or '?' in args.configs:
        # Glob pattern
        config_files = glob.glob(args.configs)
    else:
        # Comma-separated list
        config_files = [f.strip() for f in args.configs.split(',')]
    
    # Filter out non-existent files
    config_files = [f for f in config_files if os.path.exists(f)]
    
    if not config_files:
        logger.error(f"No config files found matching: {args.configs}")
        return
    
    # Parse budgets
    budgets = [int(b.strip()) for b in args.budgets.split(',')]
    
    # Parse planners
    planners = [p.strip() for p in args.planners.split(',')]
    
    logger.info(f"Config files: {config_files}")
    logger.info(f"Budgets: {budgets}")
    logger.info(f"Planners: {planners}")
    
    # Create analyzer and run
    analyzer = BudgetAnalyzer(
        config_files=config_files,
        budgets=budgets,
        planners=planners,
        n_episodes=args.n_episodes,
        max_steps_per_episode=args.max_steps,
        n_parallel=args.n_parallel,
        output_dir=args.output_dir
    )
    
    # Run analysis
    results = analyzer.run_full_analysis()
    
    # Print summary
    print("\n" + "="*60)
    print("BUDGET ANALYSIS COMPLETED")
    print("="*60)
    print(f"Results directory: {results['output_dir']}")
    print(f"CSV results: {os.path.basename(results['csv_file'])}")
    print(f"Detailed JSON: {os.path.basename(results['json_file'])}")
    print(f"Summary report: {os.path.basename(results['report_file'])}")
    print(f"Generated {len(results['plot_files'])} plots")
    print("="*60)


if __name__ == "__main__":
    main()