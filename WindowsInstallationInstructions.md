# Introduction #
The procedure for getting everything set up is simple:

  1. Download and install the JDK and the leJOS development kit
  1. Download and configure the Eclipse IDE
  1. Install the necessary Eclipse plugins (leJOS and Google)
  1. Connect to this repository

The procedure to get everything ready will take around one hour. **Please remember to read the "Important Notes" at the bottom once you finish.** If you have any questions on how to do something along the way, please leave a comment.

# Details #

First, open the homepage of this space in a different browser tab.

## Download and install prerequisites ##
Click the link on the bottom left that says "Download Java Development Kit".

https://wviaroc2014.googlecode.com/svn/wiki/1.PNG

Click the button that says "Accept License Agreement", then click to download the Windows x86 (32-bit) version. **Remember to install the 32-bit version _even if_ you are using a 64-bit operating system!**

https://wviaroc2014.googlecode.com/svn/wiki/2.PNG

Once the file finishes downloading, run the program and you should see the following screen (if you get a User Account Control message, click "Yes" or enter an administrator's password):

https://wviaroc2014.googlecode.com/svn/wiki/3.PNG

Click "Next" twice to begin installing. After a few minutes, you will see another install prompt prompting you to install the Java Runtime Environment, which is part of the JDK:

https://wviaroc2014.googlecode.com/svn/4.PNG

Click "Next". After a few more minutes, you will see this screen indicating that the install has finished:

https://wviaroc2014.googlecode.com/svn/wiki/5.PNG

At this point, you may see a lot of prompts for User Account Control and others. Just click "Yes" or "Allow" on all of these (also check "Do not show me the warning for this program again" to disable some of the prompts). After that, click "Close" to close the installation dialog.

Now, go back to the wiki homepage. Click the link that begins with "leJOS Official Website":

https://wviaroc2014.googlecode.com/svn/wiki/6.PNG

Click "Downloads" under "leJOS EV3":

https://wviaroc2014.googlecode.com/svn/wiki/7.PNG

Click "0.8.1\_beta":

https://wviaroc2014.googlecode.com/svn/wiki/8.PNG

Click the second file ("lejos\_EV3\_0.8.1-beta\_win32\_setup.exe"):

https://wviaroc2014.googlecode.com/svn/wiki/9.PNG

Once the file finishes downloading, you should see this screen:

https://wviaroc2014.googlecode.com/svn/wiki/10.PNG

Click "Next" and you will be prompted to choose the location of a 32-bit Java Development Kit. **You must have installed the 32-bit JDK, _not_ the 64-bit JDK, in order for this to work!**

https://wviaroc2014.googlecode.com/svn/wiki/11.PNG

Click "Next" twice and change the installation type from "Compact" to "Full":

https://wviaroc2014.googlecode.com/svn/wiki/12.PNG

Click "Next" three times, then click "Install". The program will now begin installing.

After a minute, the installation will complete and you will see this screen:

https://wviaroc2014.googlecode.com/svn/wiki/13.PNG

Uncheck "Launch EV3SDCard utility" and click "Finish".

**You have now finished step 1 of 4 in installing!**

## Download and configure Eclipse ##
Go back to the project homepage, then click "Download Eclipse IDE for Java Developers".

https://wviaroc2014.googlecode.com/svn/wiki/14.PNG

Click whatever mirror is next to the download icon to begin downloading the IDE:

https://wviaroc2014.googlecode.com/svn/wiki/15.PNG

Once the ZIP file finishes downloading, open it and extract it. Follow the instructions in the wizard that follows:

https://wviaroc2014.googlecode.com/svn/wiki/16.PNG

Once the extraction completes, create a Start screen tile for Eclipse by going into the Eclipse folder of the newly opened window and right click "eclipse.exe" and select "Create shortcut". After that, rename the shortcut from "eclipse.exe - Shortcut" to "Eclipse". Finally, right-click the shortcut and select "Pin to Start" You should now have a tile for Eclipse in your Start screen and Eclipse should show up in your All Apps screen.

Start Eclipse using the newly created tile. You should see a "Select Workspace" dialog like the one below:

https://wviaroc2014.googlecode.com/svn/wiki/17.PNG

Check "Use this as the default and do not ask again", then click OK.

**You have now finished step 2 of 4 in installing!**

## Install the necessary Eclipse plugins ##

In the Eclipse window, click the Help menu, then click "Install New Software":

https://wviaroc2014.googlecode.com/svn/wiki/18.PNG

Click the "Add" button, then enter "leJOS EV3" for the name (without the quotes) and enter the following URL for the location: http://lejos.sourceforge.net/tools/eclipse/plugin/ev3. Click OK, then Select All, then Next twice. Finally, choose "I accept the license terms" and click Finish. The plugin will now install.

**Note: If you see a Security Warning like the one below, click OK.**

https://wviaroc2014.googlecode.com/svn/wiki/19.PNG

When prompted to restart Eclipse, select "Yes". Eclipse will now be able to load programs onto the EV3 using the leJOS development kit you installed earlier.

Now, let's install the Google plugin so that we can connect to this repository. Follow the same instructions above to install this plugin, except use the name "Google" and the location https://dl.google.com/eclipse/plugin/4.3.

**Note: After adding the site, wait a second before clicking Select All. The installation of this plugin will take a while. After waiting around 15 minutes, watch for a Security Warning and click OK to continue with the installation.**

After the installation completes, restart Eclipse and close any dialogs that appear.

**You have now completed step 3 of 4 of installation!**

## Last step: Connect to this repository ##
Once the install completes, you should see a new Google menu in the toolbar.

https://wviaroc2014.googlecode.com/svn/wiki/20.PNG

Open this menu, then choose "Import Google Hosted Project". You will now be prompted to sign into the Google Account you used to sign into this project.

https://wviaroc2014.googlecode.com/svn/wiki/21.PNG

Once you sign in, scroll down and click "Accept". Select "wviaroc2014" from the list and choose "Next", then click "Install Subclipse". Click "Next" twice, click "I accept the licence terms, then click "Finish". **If you get a Security Warning, click OK.** Restart Eclipse when prompted.

Once Eclipse finishes restarting, close any dialog boxes that appear, and then click the Google menu and choose "Import Google Hosted Project" again. Choose "wviaroc2014", then click Next. Select all of the folders and click **Finish** (**not Next**). The files from the repository will now be downloaded to your system so that you can edit them.

**Congratulations, you have finished the installation procedure!**

## Important notes ##
Store your code in the trunk folder. Before editing any code, right click the folder and click "Team" -> "Update to HEAD" to make sure that you have the latest version of the files on your system. To commit your changes to this space, right click the source files you made changes to (or the entire trunk folder) and choose "Team" -> "Commit". Enter a brief summary of changes you made, and then click OK. **Note that merely clicking the Save button will only save your changes onto your hard drive and will NOT modify files on the repository.**