from tkinter import *
from tkinter import ttk
import subprocess
import threading
import os

class ButlerBotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Butler Bot GUI")
        self.root.geometry("750x900")
        self.root.configure(bg="#f0f0f0")
        self.process = None
        self.create_widgets()

    def create_widgets(self):
        Label(self.root, text="Butler Bot Task Manager", font=("Arial", 18, "bold"), bg="#f0f0f0").pack(pady=10)

        task_frame = Frame(self.root, bg="#f0f0f0")
        task_frame.pack(pady=10)

        Button(task_frame, text="Task 1", width=18, bg="#4caf50", fg="white", command=self.run_task1).grid(row=0, column=0, padx=10, pady=5)
        Button(task_frame, text="Task 2", width=18, bg="#4caf50", fg="white", command=self.run_task2).grid(row=1, column=0, padx=10, pady=5)

        Label(task_frame, text="Task 3 Table:", bg="#f0f0f0").grid(row=0, column=1)
        self.task3_table = ttk.Combobox(task_frame, values=["table1", "table2", "table3"])
        self.task3_table.set("table1")
        self.task3_table.grid(row=0, column=2)
        Button(task_frame, text="Task 3", bg="#2196f3", fg="white", command=self.run_task3).grid(row=0, column=3, padx=10)

        Label(task_frame, text="Task 4 Table:", bg="#f0f0f0").grid(row=1, column=1)
        self.task4_table = ttk.Combobox(task_frame, values=["table1", "table2", "table3"])
        self.task4_table.set("table1")
        self.task4_table.grid(row=1, column=2)
        Button(task_frame, text="Task 4", bg="#2196f3", fg="white", command=self.run_task4).grid(row=1, column=3, padx=10)

        Label(task_frame, text="Task 5 Tables (comma-separated):", bg="#f0f0f0").grid(row=2, column=0, columnspan=2)
        self.task5_tables = Entry(task_frame, width=30)
        self.task5_tables.insert(0, "table1,table3")
        self.task5_tables.grid(row=2, column=2)
        Button(task_frame, text="Task 5", bg="#673ab7", fg="white", command=self.run_task5).grid(row=2, column=3, padx=10)

        Label(task_frame, text="Task 6 Tables (comma-separated):", bg="#f0f0f0").grid(row=3, column=0, columnspan=2)
        self.task6_tables = Entry(task_frame, width=30)
        self.task6_tables.insert(0, "table1,table2,table3")
        self.task6_tables.grid(row=3, column=2)
        Button(task_frame, text="Task 6", bg="#9c27b0", fg="white", command=self.run_task6).grid(row=3, column=3, padx=10)

        Label(task_frame, text="Task 7 Tables (comma-separated):", bg="#f0f0f0").grid(row=4, column=0, columnspan=2)
        self.task7_tables = Entry(task_frame, width=30)
        self.task7_tables.insert(0, "table1,table2,table3")
        self.task7_tables.grid(row=4, column=2)
        Button(task_frame, text="Task 7", bg="#009688", fg="white", command=self.run_task7).grid(row=4, column=3, padx=10)

        Button(task_frame, text="🔁 Reverse Tables", bg="#607d8b", fg="white", command=self.reverse_tables).grid(row=5, column=3, padx=10, pady=5)

        # Confirm/Cancel Buttons for Kitchen and Tables 1-3
        confirm_frame = Frame(self.root, bg="#f0f0f0")
        confirm_frame.pack(pady=10)
        Label(confirm_frame, text="✅ Confirm / ❌ Cancel for Tables and Kitchen", bg="#f0f0f0", font=("Arial", 14)).pack(pady=5)

        topic_buttons = Frame(confirm_frame, bg="#f0f0f0")
        topic_buttons.pack()

        entries = ["kitchen", "table1", "table2", "table3"]
        for i, location in enumerate(entries):
            Button(topic_buttons, text=f"Confirm {location}", width=20, bg="#ff9800", fg="white", command=lambda loc=location: self.publish_topic_async("/confirm_stage", loc)).grid(row=i, column=0, padx=10, pady=2)
            Button(topic_buttons, text=f"Cancel {location}", width=20, bg="#f44336", fg="white", command=lambda loc=location: self.publish_topic_async("/cancel_task", loc)).grid(row=i, column=1, padx=10, pady=2)

        Button(self.root, text="🚩 Stop Task", bg="#9e9e9e", fg="white", command=self.stop_task).pack(pady=5)
        Button(self.root, text="🔄 Refresh Console", bg="#03a9f4", fg="white", command=self.refresh_console).pack(pady=5)

        Label(self.root, text="Console Log:", bg="#f0f0f0").pack()
        self.log = Text(self.root, height=12, width=90, bg="white")
        self.log.pack(pady=5)

    def run_task(self, script_name, args=None):
        def task_thread():
            try:
                script_path = os.path.expanduser(f"~/goat/src/butler_bot/butler_bot/{script_name}")
                cmd = ["python3", script_path]
                if args:
                    cmd += args
                self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                for line in self.process.stdout:
                    self.log.insert(END, line)
                    self.log.see(END)
            except Exception as e:
                self.log.insert(END, f"Error: {str(e)}\n")

        threading.Thread(target=task_thread, daemon=True).start()

    def stop_task(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.log.insert(END, "\n🚩 Task has been stopped.\n")
        else:
            self.log.insert(END, "\n⚠️ No active task to stop.\n")

    def publish_topic_async(self, topic, value):
        def publish_thread():
            try:
                subprocess.run(["ros2", "topic", "pub", topic, "std_msgs/msg/String", f"data: '{value}'", "--once"], check=False)
                self.log.insert(END, f"Published '{value}' to {topic}\n")
                self.log.see(END)
            except Exception as e:
                self.log.insert(END, f"Failed to publish to {topic}: {e}\n")
        threading.Thread(target=publish_thread, daemon=True).start()

    def run_task1(self):
        self.run_task("butler_task1_node.py")

    def run_task2(self):
        self.run_task("butler_task2_node.py")

    def run_task3(self):
        table = self.task3_table.get()
        self.run_task("butler_task3_node.py", ["--ros-args", "-p", f"table:={table}"])

    def run_task4(self):
        table = self.task4_table.get()
        self.run_task("butler_task4_node.py", ["--ros-args", "-p", f"table:={table}"])

    def run_task5(self):
        tables = self.task5_tables.get().replace(" ", "")
        self.run_task("butler_task5_node.py", ["--ros-args", "-p", f"tables:=[{tables}]"])


    def run_task6(self):
        tables = self.task6_tables.get().replace(" ", "")
        self.run_task("butler_task6_node.py", ["--ros-args", "-p", f"tables:=[{tables}]"])

    def run_task7(self):
        tables = self.task7_tables.get().replace(" ", "")
        self.run_task("butler_task7_node.py", ["--ros-args", "-p", f"tables:=[{tables}]"])

    def reverse_tables(self):
        for entry in [self.task5_tables, self.task6_tables, self.task7_tables]:
            tables = entry.get().replace(" ", "").split(",")
            if len(tables) > 1:
                reversed_tables = ",".join(reversed(tables))
                entry.delete(0, END)
                entry.insert(0, reversed_tables)

        self.log.insert(END, "🔄 Tables reversed!\n")
        self.log.see(END)

    def refresh_console(self):
        self.log.delete(1.0, END)
        self.log.insert(END, "🔄 Console cleared.\n")

if __name__ == "__main__":
    root = Tk()
    app = ButlerBotGUI(root)
    root.mainloop()