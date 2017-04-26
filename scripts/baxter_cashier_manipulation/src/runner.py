#!/usr/bin/env python
# [camera/camera_nodelet_manager-1] process has died
from Tkinter import Tk, Label, Button, Entry, StringVar, DISABLED, NORMAL, END, W, E
import Tkinter
import subprocess
from subprocess import Popen, PIPE
import threading
import os

class Node(threading.Thread):
  def __init__(self, command):
    threading.Thread.__init__(self)
    self.process = None
    self._command = command
    self.is_running = False

  def stop(self):
    self.process.terminate()
    self.process = None
    self.is_running = False

  def b(self):
    self.is_running = True
    self.process = Popen(self._command, stdout=PIPE, stderr=PIPE, shell=True)
    self.stdout, self.stderr = self.process.communicate()

  def run(self):
    self.is_running = True
    self.process = Popen(self._command, stdout=PIPE, stderr=PIPE,  shell=True)
    self.stdout, self.stderr = self.process.communicate()


class RunTool:
  def __init__(self, master):
    self.camera_driver = None
    self.skeleton_tracker = None
    self.moveit = None
    self.cashier = None
    self.shopkeeper = None

    def create_label(text):
      label_text = StringVar()
      label_text.set(text)
      label = Label(master, textvariable=label_text)
      return label

    self.master = master
    master.title("Baxter Cashier")

    self.camera_driver_label = create_label("Camera Driver")
    self.skeleton_tracker_label = create_label("Skeleton Tracker")
    self.moveit_label = create_label("MoveIt!")
    self.camera_calibration_tool_label = create_label("Camera Calibrator Tool")
    self.cashier_label = create_label("Cashier")
    self.shopkeeper_label = create_label("Shopkeeper")

    labels = [self.camera_driver_label,
              self.skeleton_tracker_label,
              self.moveit_label,
              self.camera_calibration_tool_label,
              self.cashier_label,
              self.shopkeeper_label]

    self.start_camera_driver = Button(master, text="Start", command=self.camera_driver_callback, width=20)
    self.start_skeleton_tracker = Button(master, text="Start", command=self.skeleton_tracker_callback, width=20)
    self.start_moveit = Button(master, text="Start", command=self.moveit_callback, width=20)
    self.start_calibration_tool = Button(master, text="Start", command=self.camera_calibrator_tool_callback, width=20)
    self.start_cashier = Button(master, text="Start", command=self.cashier_callback, width=20)
    self.start_shopkeeper = Button(master, text="Start", command=self.shopkeeper_callback, width=20)

    self.buttons = [self.start_camera_driver,
                    self.start_skeleton_tracker,
                    self.start_moveit,
                    self.start_calibration_tool,
                    self.start_cashier,
                    self.start_shopkeeper]

    label_button_pairs = zip(labels, self.buttons)
    row = 0
    for label, button in label_button_pairs:
      label.grid(row=row, column=0, sticky=Tkinter.W)
      button.grid(row=row, column=1)
      row += 1

    self.start_all_button = Button(master,
                                   text="Start All",
                                   command=self.start_all_callback,
                                   width=20)

    self.stop_all_button = Button(master,
                                  text="Stop All",
                                  command=self.stop_all_callback,
                                  width=20,
                                  state=DISABLED)

    self.start_all_button.grid(row=row, column=0)
    self.stop_all_button.grid(row=row, column=1)

  def _change_buttons_text(self, text):
    for button in self.buttons:
      button.configure(text=text)

  def start_all_callback(self):
    self.start_all_button.config(state=DISABLED)
    self.stop_all_button.config(state=NORMAL)
    self._change_buttons_text("Stop")
    print("Start all")

  def stop_all_callback(self):
    self.stop_all_button.config(state=DISABLED)
    self.start_all_button.config(state=NORMAL)
    self._change_buttons_text("Start")
    print("Stop all")

  def camera_driver_callback(self):
    if self.camera_driver is not None and self.camera_driver.is_running:
      self.camera_driver.stop()
      self.camera_driver = None
      self.start_camera_driver.configure(text="Start")
      print("Camera driver stopped...")
    else:
      cmd = os.path.expanduser('~/ros_ws/baxter.sh')
      self.camera_driver = Node('{}; roslaunch baxter_cashier_perception camera_driver.launch'.format(cmd))
      self.camera_driver.start()
      self.start_camera_driver.configure(text="Stop")
      print("Camera driver started...")

  def skeleton_tracker_callback(self):
    if self.skeleton_tracker is not None and self.skeleton_tracker.is_running:
      self.skeleton_tracker.stop()
      self.skeleton_tracker = None
      self.start_skeleton_tracker.configure(text="Start")
      print("Camera driver stopped...")
    else:
      cmd = os.path.expanduser('~/ros_ws/baxter.sh')
      self.skeleton_tracker = Node('{};  roslaunch baxter_cashier_perception body_tracker.launch'.format(cmd))
      self.skeleton_tracker.start()
      self.start_skeleton_tracker.configure(text="Stop")
      print("Camera driver started...")

  def moveit_callback(self):
    if self.moveit is not None and self.moveit.is_running:
      self.moveit.stop()
      self.moveit = None
      self.start_moveit.configure(text="Start")
      print("Camera driver stopped...")
    else:
      cmd = os.path.expanduser('~/ros_ws/baxter.sh')
      self.moveit = Node('{}; roslaunch baxter_cashier_manipulation moveit.launch'.format(cmd))
      self.moveit.start()
      self.start_moveit.configure(text="Stop")
      print("Camera driver started...")

  def camera_calibrator_tool_callback(self):
      pass

  def cashier_callback(self):
    if self.cashier is not None and self.cashier.is_running:
      self.cashier.stop()
      self.cashier = None
      self.start_cashier.configure(text="Start")
      print("Camera driver stopped...")
    else:
      cmd = os.path.expanduser('~/ros_ws/baxter.sh')
      self.cashier = Node('{}; roslaunch baxter_cashier_manipulation cashier.launch'.format(cmd))
      self.cashier.start()
      self.start_cashier.configure(text="Stop")
      print("Camera driver started...")

  def shopkeeper_callback(self):
    if self.shopkeeper is not None and self.shopkeeper.is_running:
      self.shopkeeper.stop()
      self.shopkeeper = None
      self.start_shopkeeper.configure(text="Start")
      print("Camera driver stopped...")
    else:
      cmd = os.path.expanduser('~/ros_ws/baxter.sh')
      self.shopkeeper = Node('{}; roslaunch manipulation shopkeeper_nk.launch'.format(cmd))
      self.shopkeeper.start()
      self.start_shopkeeper.configure(text="Stop")
      print("Camera driver started...")

root = Tk()
my_gui = RunTool(root)
root.mainloop()
